/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2001-2002 NaN Holding BV. All rights reserved. */

/** \file
 * \ingroup wm
 *
 * User level access for blend file read/write, file-history and user-preferences
 * (including relevant operators).
 */

/* Placed up here because of crappy WINSOCK stuff. */
#include <errno.h>
#include <fcntl.h> /* for open flags (O_BINARY, O_RDONLY). */
#include <stddef.h>
#include <string.h>

#ifdef WIN32
/* Need to include windows.h so _WIN32_IE is defined. */
#  include <windows.h>
#  ifndef _WIN32_IE
/* Minimal requirements for SHGetSpecialFolderPath on MINGW MSVC has this defined already. */
#    define _WIN32_IE 0x0400
#  endif
/* For SHGetSpecialFolderPath, has to be done before BLI_winstuff
 * because 'near' is disabled through BLI_windstuff */
#  include "BLI_winstuff.h"
#  include <shlobj.h>
#endif

#include "MEM_CacheLimiterC-Api.h"
#include "MEM_guardedalloc.h"

#include "BLI_blenlib.h"
#include "BLI_fileops_types.h"
#include "BLI_filereader.h"
#include "BLI_linklist.h"
#include "BLI_math.h"
#include "BLI_system.h"
#include "BLI_threads.h"
#include "BLI_timer.h"
#include "BLI_utildefines.h"
#include BLI_SYSTEM_PID_H

#include "PIL_time.h"

#include "BLO_readfile.h"
#include "BLT_translation.h"

#include "BLF_api.h"

#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_types.h"
#include "DNA_userdef_types.h"
#include "DNA_windowmanager_types.h"
#include "DNA_workspace_types.h"

#include "AS_asset_library.h"

#include "BKE_addon.h"
#include "BKE_appdir.h"
#include "BKE_autoexec.h"
#include "BKE_blender.h"
#include "BKE_blender_undo.h"
#include "BKE_blendfile.h"
#include "BKE_callbacks.h"
#include "BKE_context.h"
#include "BKE_global.h"
#include "BKE_idprop.h"
#include "BKE_lib_id.h"
#include "BKE_lib_override.h"
#include "BKE_lib_remap.h"
#include "BKE_main.h"
#include "BKE_main_namemap.h"
#include "BKE_multires.h"
#include "BKE_packedFile.h"
#include "BKE_paint.h"
#include "BKE_report.h"
#include "BKE_scene.h"
#include "BKE_screen.h"
#include "BKE_sound.h"
#include "BKE_undo_system.h"
#include "BKE_workspace.h"

#include "BLO_undofile.h" /* to save from an undo memfile */
#include "BLO_writefile.h"

#include "RNA_access.h"
#include "RNA_define.h"

#include "IMB_imbuf.h"
#include "IMB_imbuf_types.h"
#include "IMB_thumbs.h"

#include "ED_asset.h"
#include "ED_datafiles.h"
#include "ED_fileselect.h"
#include "ED_image.h"
#include "ED_outliner.h"
#include "ED_render.h"
#include "ED_screen.h"
#include "ED_undo.h"
#include "ED_util.h"
#include "ED_view3d.h"
#include "ED_view3d_offscreen.h"

#include "GHOST_C-api.h"
#include "GHOST_Path-api.hh"

#include "GPU_context.h"

#include "UI_interface.h"
#include "UI_resources.h"
#include "UI_view2d.h"

/* only to report a missing engine */
#include "RE_engine.h"

#ifdef WITH_PYTHON
#  include "BPY_extern_python.h"
#  include "BPY_extern_run.h"
#endif

#include "DEG_depsgraph.h"

#include "WM_api.h"
#include "WM_message.h"
#include "WM_toolsystem.h"
#include "WM_types.h"

#include "wm.h"
#include "wm_event_system.h"
#include "wm_files.h"
#include "wm_window.h"

#include "CLG_log.h"

static RecentFile *wm_file_history_find(const char *filepath);
static void wm_history_file_free(RecentFile *recent);
static void wm_history_files_free(void);
static void wm_history_file_update(void);
static void wm_history_file_write(void);

static void wm_test_autorun_revert_action_exec(bContext *C);

static CLG_LogRef LOG = {"wm.files"};

/* -------------------------------------------------------------------- */
/** \name Misc Utility Functions
 * \{ */

void WM_file_tag_modified(void)
{
  wmWindowManager *wm = static_cast<wmWindowManager *>(G_MAIN->wm.first);
  if (wm->file_saved) {
    wm->file_saved = 0;
    /* notifier that data changed, for save-over warning or header */
    WM_main_add_notifier(NC_WM | ND_DATACHANGED, nullptr);
  }
}

bool wm_file_or_session_data_has_unsaved_changes(const Main *bmain, const wmWindowManager *wm)
{
  return !wm->file_saved || ED_image_should_save_modified(bmain) ||
         AS_asset_library_has_any_unsaved_catalogs();
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Window Matching for File Reading
 * \{ */

/**
 * To be able to read files without windows closing, opening, moving
 * we try to prepare for worst case:
 * - active window gets active screen from file
 * - restoring the screens from non-active windows
 * Best case is all screens match, in that case they get assigned to proper window.
 */
static void wm_window_match_init(bContext *C, ListBase *wmlist)
{
  *wmlist = G_MAIN->wm;

  wmWindow *active_win = CTX_wm_window(C);

  /* first wrap up running stuff */
  /* code copied from wm_init_exit.cc */
  LISTBASE_FOREACH (wmWindowManager *, wm, wmlist) {
    WM_jobs_kill_all(wm);

    LISTBASE_FOREACH (wmWindow *, win, &wm->windows) {
      CTX_wm_window_set(C, win); /* needed by operator close callbacks */
      WM_event_remove_handlers(C, &win->handlers);
      WM_event_remove_handlers(C, &win->modalhandlers);
      ED_screen_exit(C, win, WM_window_get_active_screen(win));
    }

    /* NOTE(@ideasman42): Clear the message bus so it's always cleared on file load.
     * Otherwise it's cleared when "Load UI" is set (see #USER_FILENOUI & #wm_close_and_free).
     * However it's _not_ cleared when the UI is kept. This complicates use from add-ons
     * which can re-register subscribers on file-load. To support this use case,
     * it's best to have predictable behavior - always clear. */
    if (wm->message_bus != nullptr) {
      WM_msgbus_destroy(wm->message_bus);
      wm->message_bus = nullptr;
    }
  }

  BLI_listbase_clear(&G_MAIN->wm);
  if (G_MAIN->name_map != nullptr) {
    /* NOTE: UI IDs are assumed to be only local data-blocks, so no need to call
     * #BKE_main_namemap_clear here. */
    BKE_main_namemap_destroy(&G_MAIN->name_map);
  }

  /* reset active window */
  CTX_wm_window_set(C, active_win);

  /* XXX Hack! We have to clear context menu here, because removing all modalhandlers
   * above frees the active menu (at least, in the 'startup splash' case),
   * causing use-after-free error in later handling of the button callbacks in UI code
   * (see ui_apply_but_funcs_after()).
   * Tried solving this by always nullptr-ing context's menu when setting wm/win/etc.,
   * but it broke popups refreshing (see #47632),
   * so for now just handling this specific case here. */
  CTX_wm_menu_set(C, nullptr);

  ED_editors_exit(G_MAIN, true);

  /* Asset loading is done by the UI/editors and they keep pointers into it. So make sure to clear
   * it after UI/editors. */
  ED_assetlist_storage_exit();
  AS_asset_libraries_exit();
}

static void wm_window_substitute_old(wmWindowManager *oldwm,
                                     wmWindowManager *wm,
                                     wmWindow *oldwin,
                                     wmWindow *win)
{
  win->ghostwin = oldwin->ghostwin;
  win->gpuctx = oldwin->gpuctx;
  win->active = oldwin->active;
  if (win->active) {
    wm->winactive = win;
  }
  if (oldwm->windrawable == oldwin) {
    oldwm->windrawable = nullptr;
    wm->windrawable = win;
  }

  /* File loading in background mode still calls this. */
  if (!G.background) {
    /* Pointer back. */
    GHOST_SetWindowUserData(static_cast<GHOST_WindowHandle>(win->ghostwin), win);
  }

  oldwin->ghostwin = nullptr;
  oldwin->gpuctx = nullptr;

  win->eventstate = oldwin->eventstate;
  win->event_last_handled = oldwin->event_last_handled;
  oldwin->eventstate = nullptr;
  oldwin->event_last_handled = nullptr;

  /* Ensure proper screen re-scaling. */
  win->sizex = oldwin->sizex;
  win->sizey = oldwin->sizey;
  win->posx = oldwin->posx;
  win->posy = oldwin->posy;
}

static void wm_window_match_keep_current_wm(const bContext *C,
                                            ListBase *current_wm_list,
                                            const bool load_ui,
                                            ListBase *r_new_wm_list)
{
  Main *bmain = CTX_data_main(C);
  wmWindowManager *wm = static_cast<wmWindowManager *>(current_wm_list->first);
  bScreen *screen = nullptr;

  /* match oldwm to new dbase, only old files */
  wm->initialized &= ~WM_WINDOW_IS_INIT;

  /* when loading without UI, no matching needed */
  if (load_ui && (screen = CTX_wm_screen(C))) {
    LISTBASE_FOREACH (wmWindow *, win, &wm->windows) {
      WorkSpace *workspace;

      BKE_workspace_layout_find_global(bmain, screen, &workspace);
      BKE_workspace_active_set(win->workspace_hook, workspace);
      win->scene = CTX_data_scene(C);

      /* all windows get active screen from file */
      if (screen->winid == 0) {
        WM_window_set_active_screen(win, workspace, screen);
      }
      else {
        WorkSpaceLayout *layout_old = WM_window_get_active_layout(win);
        WorkSpaceLayout *layout_new = ED_workspace_layout_duplicate(
            bmain, workspace, layout_old, win);

        WM_window_set_active_layout(win, workspace, layout_new);
      }

      bScreen *win_screen = WM_window_get_active_screen(win);
      win_screen->winid = win->winid;
    }
  }

  /* we'll be using the current wm list directly; make sure
   * the names are validated and in the name map. */
  LISTBASE_FOREACH (wmWindowManager *, wm_item, current_wm_list) {
    BKE_main_namemap_get_name(bmain, &wm_item->id, wm_item->id.name + 2);
  }

  *r_new_wm_list = *current_wm_list;
}

static void wm_window_match_replace_by_file_wm(bContext *C,
                                               ListBase *current_wm_list,
                                               ListBase *readfile_wm_list,
                                               ListBase *r_new_wm_list)
{
  wmWindowManager *oldwm = static_cast<wmWindowManager *>(current_wm_list->first);
  /* will become our new WM */
  wmWindowManager *wm = static_cast<wmWindowManager *>(readfile_wm_list->first);

  /* Support window-manager ID references being held between file load operations by keeping
   * #Main.wm.first memory address in-place, while swapping all of it's contents.
   *
   * This is needed so items such as key-maps can be held by an add-on,
   * without it pointing to invalid memory, see: #86431 */
  {
    /* Referencing the window-manager pointer from elsewhere in the file is highly unlikely
     * however it's possible with ID-properties & animation-drivers.
     * At some point we could check on disallowing this since it doesn't seem practical. */
    Main *bmain = G_MAIN;
    BLI_assert(bmain->relations == nullptr);
    BKE_libblock_remap(bmain, wm, oldwm, ID_REMAP_SKIP_INDIRECT_USAGE | ID_REMAP_SKIP_USER_CLEAR);

    /* Maintain the undo-depth between file loads. Useful so Python can perform
     * nested operator calls that exit with the proper undo-depth. */
    wm->op_undo_depth = oldwm->op_undo_depth;

    /* Simple pointer swapping step. */
    BLI_remlink(current_wm_list, oldwm);
    BLI_remlink(readfile_wm_list, wm);
    SWAP(wmWindowManager, *oldwm, *wm);
    SWAP(wmWindowManager *, oldwm, wm);
    BLI_addhead(current_wm_list, oldwm);
    BLI_addhead(readfile_wm_list, wm);

    /* Don't leave the old pointer in the context. */
    CTX_wm_manager_set(C, wm);
  }

  bool has_match = false;

  /* this code could move to setup_appdata */

  /* preserve key configurations in new wm, to preserve their keymaps */
  wm->keyconfigs = oldwm->keyconfigs;
  wm->addonconf = oldwm->addonconf;
  wm->defaultconf = oldwm->defaultconf;
  wm->userconf = oldwm->userconf;

  BLI_listbase_clear(&oldwm->keyconfigs);
  oldwm->addonconf = nullptr;
  oldwm->defaultconf = nullptr;
  oldwm->userconf = nullptr;

  /* ensure making new keymaps and set space types */
  wm->initialized = 0;
  wm->winactive = nullptr;

  /* Clearing drawable of before deleting any context
   * to avoid clearing the wrong wm. */
  wm_window_clear_drawable(oldwm);

  /* Only first `wm` in list has GHOST-windows. */
  LISTBASE_FOREACH (wmWindow *, win, &wm->windows) {
    LISTBASE_FOREACH (wmWindow *, oldwin, &oldwm->windows) {
      if (oldwin->winid == win->winid) {
        has_match = true;

        wm_window_substitute_old(oldwm, wm, oldwin, win);
      }
    }
  }
  /* make sure at least one window is kept open so we don't lose the context, check #42303 */
  if (!has_match) {
    wm_window_substitute_old(oldwm,
                             wm,
                             static_cast<wmWindow *>(oldwm->windows.first),
                             static_cast<wmWindow *>(wm->windows.first));
  }

  wm_close_and_free_all(C, current_wm_list);

  *r_new_wm_list = *readfile_wm_list;
}

/**
 * Match old WM with new, 4 cases:
 * 1) No current WM, no WM in file: Make new default.
 * 2) No current WM, but WM in file: Keep current WM, do nothing else.
 * 3) Current WM, but not in file: Keep current WM, update windows with screens from file.
 * 4) Current WM, and WM in file: Try to keep current GHOST windows, use WM from file.
 *
 * \param r_new_wm_list: Return argument for the wm list to be used from now on.
 */
static void wm_window_match_do(bContext *C,
                               ListBase *current_wm_list,
                               ListBase *readfile_wm_list,
                               ListBase *r_new_wm_list)
{
  if (BLI_listbase_is_empty(current_wm_list)) {
    /* case 1 */
    if (BLI_listbase_is_empty(readfile_wm_list)) {
      Main *bmain = CTX_data_main(C);
      /* Neither current, no newly read file have a WM -> add the default one. */
      wm_add_default(bmain, C);
      *r_new_wm_list = bmain->wm;
    }
    /* case 2 */
    else {
      *r_new_wm_list = *readfile_wm_list;
    }
  }
  else {
    /* case 3 */
    if (BLI_listbase_is_empty(readfile_wm_list)) {
      /* We've read file without wm, keep current one entirely alive.
       * Happens when reading pre 2.5 files (no WM back then) */
      wm_window_match_keep_current_wm(
          C, current_wm_list, (G.fileflags & G_FILE_NO_UI) == 0, r_new_wm_list);
    }
    /* case 4 */
    else {
      wm_window_match_replace_by_file_wm(C, current_wm_list, readfile_wm_list, r_new_wm_list);
    }
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Preferences Initialization & Versioning
 * \{ */

static void wm_gpu_backend_override_from_userdef(void)
{
  /* Check if GPU backend is already set from the command line arguments. The command line
   * arguments have higher priority than user preferences. */
  if (GPU_backend_type_selection_is_overridden()) {
    return;
  }

  GPU_backend_type_selection_set_override(eGPUBackendType(U.gpu_backend));
}

/**
 * In case #UserDef was read, re-initialize values that depend on it.
 */
static void wm_init_userdef(Main *bmain)
{
  /* Not versioning, just avoid errors. */
#ifndef WITH_CYCLES
  BKE_addon_remove_safe(&U.addons, "cycles");
#else
  UNUSED_VARS(BKE_addon_remove_safe);
#endif

  UI_init_userdef();

  /* needed so loading a file from the command line respects user-pref #26156. */
  SET_FLAG_FROM_TEST(G.fileflags, U.flag & USER_FILENOUI, G_FILE_NO_UI);

  /* set the python auto-execute setting from user prefs */
  /* enabled by default, unless explicitly enabled in the command line which overrides */
  if ((G.f & G_FLAG_SCRIPT_OVERRIDE_PREF) == 0) {
    SET_FLAG_FROM_TEST(G.f, (U.flag & USER_SCRIPT_AUTOEXEC_DISABLE) == 0, G_FLAG_SCRIPT_AUTOEXEC);
  }

  MEM_CacheLimiter_set_maximum(size_t(U.memcachelimit) * 1024 * 1024);
  BKE_sound_init(bmain);

  /* Update the temporary directory from the preferences or fallback to the system default. */
  BKE_tempdir_init(U.tempdir);

  /* Update input device preference. */
  WM_init_input_devices();

  BLO_sanitize_experimental_features_userpref_blend(&U);

  wm_gpu_backend_override_from_userdef();
  GPU_backend_type_selection_detect();
}

/* return codes */
#define BKE_READ_EXOTIC_FAIL_PATH -3   /* file format is not supported */
#define BKE_READ_EXOTIC_FAIL_FORMAT -2 /* file format is not supported */
#define BKE_READ_EXOTIC_FAIL_OPEN -1   /* Can't open the file */
#define BKE_READ_EXOTIC_OK_BLEND 0     /* .blend file */
#if 0
#  define BKE_READ_EXOTIC_OK_OTHER 1 /* other supported formats */
#endif

/** \} */

/* -------------------------------------------------------------------- */
/** \name Read Exotic File Formats
 *
 * Currently only supports '.blend' files,
 * we could support registering other file formats and their loaders.
 * \{ */

/* intended to check for non-blender formats but for now it only reads blends */
static int wm_read_exotic(const char *name)
{
  /* make sure we're not trying to read a directory.... */

  int namelen = strlen(name);
  if (namelen > 0 && ELEM(name[namelen - 1], '/', '\\')) {
    return BKE_READ_EXOTIC_FAIL_PATH;
  }

  /* open the file. */
  const int filedes = BLI_open(name, O_BINARY | O_RDONLY, 0);
  if (filedes == -1) {
    return BKE_READ_EXOTIC_FAIL_OPEN;
  }

  FileReader *rawfile = BLI_filereader_new_file(filedes);
  if (rawfile == nullptr) {
    return BKE_READ_EXOTIC_FAIL_OPEN;
  }

  /* read the header (7 bytes are enough to identify all known types). */
  char header[7];
  if (rawfile->read(rawfile, header, sizeof(header)) != sizeof(header)) {
    rawfile->close(rawfile);
    return BKE_READ_EXOTIC_FAIL_FORMAT;
  }
  rawfile->seek(rawfile, 0, SEEK_SET);

  /* check for uncompressed .blend */
  if (STREQLEN(header, "BLENDER", 7)) {
    rawfile->close(rawfile);
    return BKE_READ_EXOTIC_OK_BLEND;
  }

  /* check for compressed .blend */
  FileReader *compressed_file = nullptr;
  if (BLI_file_magic_is_gzip(header)) {
    /* In earlier versions of Blender (before 3.0), compressed files used `Gzip` instead of `Zstd`.
     * While these files will no longer be written, there still needs to be reading support. */
    compressed_file = BLI_filereader_new_gzip(rawfile);
  }
  else if (BLI_file_magic_is_zstd(header)) {
    compressed_file = BLI_filereader_new_zstd(rawfile);
  }

  /* If a compression signature matches, try decompressing the start and check if it's a .blend */
  if (compressed_file != nullptr) {
    size_t len = compressed_file->read(compressed_file, header, sizeof(header));
    compressed_file->close(compressed_file);
    if (len == sizeof(header) && STREQLEN(header, "BLENDER", 7)) {
      return BKE_READ_EXOTIC_OK_BLEND;
    }
  }
  else {
    rawfile->close(rawfile);
  }

  /* Add check for future file formats here. */

  return BKE_READ_EXOTIC_FAIL_FORMAT;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Read Blend-File Shared Utilities
 * \{ */

void WM_file_autoexec_init(const char *filepath)
{
  if (G.f & G_FLAG_SCRIPT_OVERRIDE_PREF) {
    return;
  }

  if (G.f & G_FLAG_SCRIPT_AUTOEXEC) {
    char dirpath[FILE_MAX];
    BLI_split_dir_part(filepath, dirpath, sizeof(dirpath));
    if (BKE_autoexec_match(dirpath)) {
      G.f &= ~G_FLAG_SCRIPT_AUTOEXEC;
    }
  }
}

void wm_file_read_report(bContext *C, Main *bmain)
{
  ReportList *reports = nullptr;
  LISTBASE_FOREACH (Scene *, scene, &bmain->scenes) {
    if (scene->r.engine[0] &&
        BLI_findstring(&R_engines, scene->r.engine, offsetof(RenderEngineType, idname)) ==
            nullptr) {
      if (reports == nullptr) {
        reports = CTX_wm_reports(C);
      }

      BKE_reportf(reports,
                  RPT_ERROR,
                  "Engine '%s' not available for scene '%s' (an add-on may need to be installed "
                  "or enabled)",
                  scene->r.engine,
                  scene->id.name + 2);
    }
  }

  if (reports) {
    if (!G.background) {
      WM_report_banner_show();
    }
  }
}

/**
 * Logic shared between #WM_file_read & #wm_homefile_read,
 * call before loading a file.
 * \note In the case of #WM_file_read the file may fail to load.
 * Change here shouldn't cause user-visible changes in that case.
 */
static void wm_file_read_pre(bool use_data, bool /*use_userdef*/)
{
  if (use_data) {
    BLI_timer_on_file_load();
  }

  /* Always do this as both startup and preferences may have loaded in many font's
   * at a different zoom level to the file being loaded. */
  UI_view2d_zoom_cache_reset();

  ED_preview_restart_queue_free();
}

/**
 * Parameters for #wm_file_read_post, also used for deferred initialization.
 */
struct wmFileReadPost_Params {
  uint use_data : 1;
  uint use_userdef : 1;

  uint is_startup_file : 1;
  uint is_factory_startup : 1;
  uint reset_app_template : 1;

  /* Used by #wm_homefile_read_post */
  uint success : 1;
  uint is_alloc : 1;
};

/**
 * Logic shared between #WM_file_read & #wm_homefile_read,
 * updates to make after reading a file.
 */
static void wm_file_read_post(bContext *C, const struct wmFileReadPost_Params *params)
{
  wmWindowManager *wm = CTX_wm_manager(C);

  const bool use_data = params->use_data;
  const bool use_userdef = params->use_userdef;
  const bool is_startup_file = params->is_startup_file;
  const bool is_factory_startup = params->is_factory_startup;
  const bool reset_app_template = params->reset_app_template;

  bool addons_loaded = false;

  if (use_data) {
    if (!G.background) {
      /* remove windows which failed to be added via WM_check */
      wm_window_ghostwindows_remove_invalid(C, wm);
    }
    CTX_wm_window_set(C, static_cast<wmWindow *>(wm->windows.first));
  }

#ifdef WITH_PYTHON
  if (is_startup_file) {
    /* On startup (by default), Python won't have been initialized.
     *
     * The following block handles data & preferences being reloaded
     * which requires resetting some internal variables. */
    if (CTX_py_init_get(C)) {
      bool reset_all = use_userdef;
      if (use_userdef || reset_app_template) {
        /* Only run when we have a template path found. */
        if (BKE_appdir_app_template_any()) {
          const char *imports[] = {"bl_app_template_utils", nullptr};
          BPY_run_string_eval(C, imports, "bl_app_template_utils.reset()");
          reset_all = true;
        }
      }
      if (reset_all) {
        const char *imports[] = {"bpy", "addon_utils", nullptr};
        BPY_run_string_exec(
            C,
            imports,
            /* Refresh scripts as the preferences may have changed the user-scripts path.
             *
             * This is needed when loading settings from the previous version,
             * otherwise the script path stored in the preferences would be ignored. */
            "bpy.utils.refresh_script_paths()\n"
            /* Sync add-ons, these may have changed from the defaults. */
            "addon_utils.reset_all()");
      }
      if (use_data) {
        BPY_python_reset(C);
      }
      addons_loaded = true;
    }
  }
  else {
    /* run any texts that were loaded in and flagged as modules */
    if (use_data) {
      BPY_python_reset(C);
    }
    addons_loaded = true;
  }
#else
  UNUSED_VARS(is_startup_file, reset_app_template);
#endif /* WITH_PYTHON */

  Main *bmain = CTX_data_main(C);

  if (use_userdef) {
    if (is_factory_startup) {
      BKE_callback_exec_null(bmain, BKE_CB_EVT_LOAD_FACTORY_USERDEF_POST);
    }
  }

  if (is_factory_startup && BLT_translate_new_dataname()) {
    /* Translate workspace names */
    LISTBASE_FOREACH_MUTABLE (WorkSpace *, workspace, &bmain->workspaces) {
      BKE_libblock_rename(
          bmain, &workspace->id, CTX_DATA_(BLT_I18NCONTEXT_ID_WORKSPACE, workspace->id.name + 2));
    }
  }

  if (use_data) {
    /* important to do before nullptr'ing the context */
    BKE_callback_exec_null(bmain, BKE_CB_EVT_VERSION_UPDATE);
    if (is_factory_startup) {
      BKE_callback_exec_null(bmain, BKE_CB_EVT_LOAD_FACTORY_STARTUP_POST);
    }
  }

  if (use_data) {
    WM_operatortype_last_properties_clear_all();

    /* After load post, so for example the driver namespace can be filled
     * before evaluating the depsgraph. */
    wm_event_do_depsgraph(C, true);

    ED_editors_init(C);

#if 1
    WM_event_add_notifier(C, NC_WM | ND_FILEREAD, nullptr);
#else
    WM_msg_publish_static(CTX_wm_message_bus(C), WM_MSG_STATICTYPE_FILE_READ);
#endif
  }

  /* report any errors.
   * currently disabled if addons aren't yet loaded */
  if (addons_loaded) {
    wm_file_read_report(C, bmain);
  }

  if (use_data) {
    if (!G.background) {
      if (wm->undo_stack == nullptr) {
        wm->undo_stack = BKE_undosys_stack_create();
      }
      else {
        BKE_undosys_stack_clear(wm->undo_stack);
      }
      BKE_undosys_stack_init_from_main(wm->undo_stack, bmain);
      BKE_undosys_stack_init_from_context(wm->undo_stack, C);
    }
  }

  if (use_data) {
    if (!G.background) {
      /* in background mode this makes it hard to load
       * a blend file and do anything since the screen
       * won't be set to a valid value again */
      CTX_wm_window_set(C, nullptr); /* exits queues */

      /* Ensure auto-run action is not used from a previous blend file load. */
      wm_test_autorun_revert_action_set(nullptr, nullptr);

      /* Ensure tools are registered. */
      WM_toolsystem_init(C);
    }
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Read Main Blend-File API
 * \{ */

static void file_read_reports_finalize(BlendFileReadReport *bf_reports)
{
  double duration_whole_minutes, duration_whole_seconds;
  double duration_libraries_minutes, duration_libraries_seconds;
  double duration_lib_override_minutes, duration_lib_override_seconds;
  double duration_lib_override_resync_minutes, duration_lib_override_resync_seconds;
  double duration_lib_override_recursive_resync_minutes,
      duration_lib_override_recursive_resync_seconds;

  BLI_math_time_seconds_decompose(bf_reports->duration.whole,
                                  nullptr,
                                  nullptr,
                                  &duration_whole_minutes,
                                  &duration_whole_seconds,
                                  nullptr);
  BLI_math_time_seconds_decompose(bf_reports->duration.libraries,
                                  nullptr,
                                  nullptr,
                                  &duration_libraries_minutes,
                                  &duration_libraries_seconds,
                                  nullptr);
  BLI_math_time_seconds_decompose(bf_reports->duration.lib_overrides,
                                  nullptr,
                                  nullptr,
                                  &duration_lib_override_minutes,
                                  &duration_lib_override_seconds,
                                  nullptr);
  BLI_math_time_seconds_decompose(bf_reports->duration.lib_overrides_resync,
                                  nullptr,
                                  nullptr,
                                  &duration_lib_override_resync_minutes,
                                  &duration_lib_override_resync_seconds,
                                  nullptr);
  BLI_math_time_seconds_decompose(bf_reports->duration.lib_overrides_recursive_resync,
                                  nullptr,
                                  nullptr,
                                  &duration_lib_override_recursive_resync_minutes,
                                  &duration_lib_override_recursive_resync_seconds,
                                  nullptr);

  CLOG_INFO(
      &LOG, 0, "Blender file read in %.0fm%.2fs", duration_whole_minutes, duration_whole_seconds);
  CLOG_INFO(&LOG,
            0,
            " * Loading libraries: %.0fm%.2fs",
            duration_libraries_minutes,
            duration_libraries_seconds);
  CLOG_INFO(&LOG,
            0,
            " * Applying overrides: %.0fm%.2fs",
            duration_lib_override_minutes,
            duration_lib_override_seconds);
  CLOG_INFO(&LOG,
            0,
            " * Resyncing overrides: %.0fm%.2fs (%d root overrides), including recursive "
            "resyncs: %.0fm%.2fs)",
            duration_lib_override_resync_minutes,
            duration_lib_override_resync_seconds,
            bf_reports->count.resynced_lib_overrides,
            duration_lib_override_recursive_resync_minutes,
            duration_lib_override_recursive_resync_seconds);

  if (bf_reports->resynced_lib_overrides_libraries_count != 0) {
    for (LinkNode *node_lib = bf_reports->resynced_lib_overrides_libraries; node_lib != nullptr;
         node_lib = node_lib->next) {
      Library *library = static_cast<Library *>(node_lib->link);
      BKE_reportf(bf_reports->reports,
                  RPT_INFO,
                  "Library \"%s\" needs overrides resync",
                  library->filepath);
    }
  }

  if (bf_reports->count.missing_libraries != 0 || bf_reports->count.missing_linked_id != 0) {
    BKE_reportf(bf_reports->reports,
                RPT_WARNING,
                "%d libraries and %d linked data-blocks are missing (including %d ObjectData and "
                "%d Proxies), please check the Info and Outliner editors for details",
                bf_reports->count.missing_libraries,
                bf_reports->count.missing_linked_id,
                bf_reports->count.missing_obdata,
                bf_reports->count.missing_obproxies);
  }
  else {
    if (bf_reports->count.missing_obdata != 0 || bf_reports->count.missing_obproxies != 0) {
      CLOG_ERROR(&LOG,
                 "%d local ObjectData and %d local Object proxies are reported to be missing, "
                 "this should never happen",
                 bf_reports->count.missing_obdata,
                 bf_reports->count.missing_obproxies);
    }
  }

  if (bf_reports->resynced_lib_overrides_libraries_count != 0) {
    BKE_reportf(bf_reports->reports,
                RPT_WARNING,
                "%d libraries have overrides needing resync (auto resynced in %.0fm%.2fs),  "
                "please check the Info editor for details",
                bf_reports->resynced_lib_overrides_libraries_count,
                duration_lib_override_recursive_resync_minutes,
                duration_lib_override_recursive_resync_seconds);
  }

  if (bf_reports->count.proxies_to_lib_overrides_success != 0 ||
      bf_reports->count.proxies_to_lib_overrides_failures != 0) {
    BKE_reportf(bf_reports->reports,
                RPT_WARNING,
                "Proxies have been removed from Blender (%d proxies were automatically converted "
                "to library overrides, %d proxies could not be converted and were cleared). "
                "Consider re-saving any library .blend file with the newest Blender version",
                bf_reports->count.proxies_to_lib_overrides_success,
                bf_reports->count.proxies_to_lib_overrides_failures);
  }

  if (bf_reports->count.sequence_strips_skipped != 0) {
    BKE_reportf(bf_reports->reports,
                RPT_ERROR,
                "%d sequence strips were not read because they were in a channel larger than %d",
                bf_reports->count.sequence_strips_skipped,
                MAXSEQ);
  }

  BLI_linklist_free(bf_reports->resynced_lib_overrides_libraries, nullptr);
  bf_reports->resynced_lib_overrides_libraries = nullptr;
}

bool WM_file_read(bContext *C, const char *filepath, ReportList *reports)
{
  /* assume automated tasks with background, don't write recent file list */
  const bool do_history_file_update = (G.background == false) &&
                                      (CTX_wm_manager(C)->op_undo_depth == 0);
  bool success = false;

  const bool use_data = true;
  const bool use_userdef = false;

  /* NOTE: either #BKE_CB_EVT_LOAD_POST or #BKE_CB_EVT_LOAD_POST_FAIL must run.
   * Runs at the end of this function, don't return beforehand. */
  BKE_callback_exec_string(CTX_data_main(C), BKE_CB_EVT_LOAD_PRE, filepath);

  /* so we can get the error message */
  errno = 0;

  WM_cursor_wait(true);

  /* first try to append data from exotic file formats... */
  /* it throws error box when file doesn't exist and returns -1 */
  /* NOTE(ton): it should set some error message somewhere. */
  const int retval = wm_read_exotic(filepath);

  /* we didn't succeed, now try to read Blender file */
  if (retval == BKE_READ_EXOTIC_OK_BLEND) {
    BlendFileReadParams params{};
    params.is_startup = false;
    /* Loading preferences when the user intended to load a regular file is a security
     * risk, because the excluded path list is also loaded. Further it's just confusing
     * if a user loads a file and various preferences change. */
    params.skip_flags = BLO_READ_SKIP_USERDEF;

    BlendFileReadReport bf_reports{};
    bf_reports.reports = reports;
    bf_reports.duration.whole = PIL_check_seconds_timer();
    struct BlendFileData *bfd = BKE_blendfile_read(filepath, &params, &bf_reports);
    if (bfd != nullptr) {
      wm_file_read_pre(use_data, use_userdef);

      /* Put aside screens to match with persistent windows later,
       * also exit screens and editors. */
      ListBase wmbase;
      wm_window_match_init(C, &wmbase);

      /* This flag is initialized by the operator but overwritten on read.
       * need to re-enable it here else drivers and registered scripts won't work. */
      const int G_f_orig = G.f;

      BKE_blendfile_read_setup(C, bfd, &params, &bf_reports);

      if (G.f != G_f_orig) {
        const int flags_keep = G_FLAG_ALL_RUNTIME;
        G.f &= G_FLAG_ALL_READFILE;
        G.f = (G.f & ~flags_keep) | (G_f_orig & flags_keep);
      }

      /* #BKE_blendfile_read_result_setup sets new Main into context. */
      Main *bmain = CTX_data_main(C);

      /* match the read WM with current WM */
      wm_window_match_do(C, &wmbase, &bmain->wm, &bmain->wm);
      WM_check(C); /* opens window(s), checks keymaps */

      if (do_history_file_update) {
        wm_history_file_update();
      }

      wmFileReadPost_Params read_file_post_params{};
      read_file_post_params.use_data = use_data;
      read_file_post_params.use_userdef = use_userdef;
      read_file_post_params.is_startup_file = false;
      read_file_post_params.is_factory_startup = false;
      read_file_post_params.reset_app_template = false;
      read_file_post_params.success = true;
      read_file_post_params.is_alloc = false;
      wm_file_read_post(C, &read_file_post_params);

      bf_reports.duration.whole = PIL_check_seconds_timer() - bf_reports.duration.whole;
      file_read_reports_finalize(&bf_reports);

      success = true;
    }
  }
#if 0
  else if (retval == BKE_READ_EXOTIC_OK_OTHER) {
    BKE_undo_write(C, "Import file");
  }
#endif
  else if (retval == BKE_READ_EXOTIC_FAIL_OPEN) {
    BKE_reportf(reports,
                RPT_ERROR,
                "Cannot read file \"%s\": %s",
                filepath,
                errno ? strerror(errno) : TIP_("unable to open the file"));
  }
  else if (retval == BKE_READ_EXOTIC_FAIL_FORMAT) {
    BKE_reportf(reports, RPT_ERROR, "File format is not supported in file \"%s\"", filepath);
  }
  else if (retval == BKE_READ_EXOTIC_FAIL_PATH) {
    BKE_reportf(reports, RPT_ERROR, "File path \"%s\" invalid", filepath);
  }
  else {
    BKE_reportf(reports, RPT_ERROR, "Unknown error loading \"%s\"", filepath);
    BLI_assert_msg(0, "invalid 'retval'");
  }

  if (success == false) {
    /* remove from recent files list */
    if (do_history_file_update) {
      RecentFile *recent = wm_file_history_find(filepath);
      if (recent) {
        wm_history_file_free(recent);
        wm_history_file_write();
      }
    }
  }

  WM_cursor_wait(false);

  Main *bmain = CTX_data_main(C);
  BKE_callback_exec_string(
      bmain, success ? BKE_CB_EVT_LOAD_POST : BKE_CB_EVT_LOAD_POST_FAIL, filepath);

  BLI_assert(BKE_main_namemap_validate(bmain));

  return success;
}

static struct {
  char app_template[64];
  bool override;
} wm_init_state_app_template = {{0}};

void WM_init_state_app_template_set(const char *app_template)
{
  if (app_template) {
    STRNCPY(wm_init_state_app_template.app_template, app_template);
    wm_init_state_app_template.override = true;
  }
  else {
    wm_init_state_app_template.app_template[0] = '\0';
    wm_init_state_app_template.override = false;
  }
}

const char *WM_init_state_app_template_get(void)
{
  return wm_init_state_app_template.override ? wm_init_state_app_template.app_template : nullptr;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Read Startup & Preferences Blend-File API
 * \{ */

void wm_homefile_read_ex(bContext *C,
                         const struct wmHomeFileRead_Params *params_homefile,
                         ReportList *reports,
                         struct wmFileReadPost_Params **r_params_file_read_post)
{
#if 0 /* UNUSED, keep as this may be needed later & the comment below isn't self evident. */
  /* Context does not always have valid main pointer here. */
  Main *bmain = G_MAIN;
#endif
  ListBase wmbase;
  bool success = false;

  /* May be enabled, when the user configuration doesn't exist. */
  const bool use_data = params_homefile->use_data;
  const bool use_userdef = params_homefile->use_userdef;
  bool use_factory_settings = params_homefile->use_factory_settings;
  /* Currently this only impacts preferences as it doesn't make much sense to keep the default
   * startup open in the case the app-template doesn't happen to define its own startup.
   * Unlike preferences where we might want to only reset the app-template part of the preferences
   * so as not to reset the preferences for all other Blender instances, see: #96427. */
  const bool use_factory_settings_app_template_only =
      params_homefile->use_factory_settings_app_template_only;
  const bool use_empty_data = params_homefile->use_empty_data;
  const char *filepath_startup_override = params_homefile->filepath_startup_override;
  const char *app_template_override = params_homefile->app_template_override;

  bool filepath_startup_is_factory = true;
  char filepath_startup[FILE_MAX];
  char filepath_userdef[FILE_MAX];

  /* When 'app_template' is set:
   * '{BLENDER_USER_CONFIG}/{app_template}' */
  char app_template_system[FILE_MAX];
  /* When 'app_template' is set:
   * '{BLENDER_SYSTEM_SCRIPTS}/startup/bl_app_templates_system/{app_template}' */
  char app_template_config[FILE_MAX];

  eBLOReadSkip skip_flags = eBLOReadSkip(0);

  if (use_data == false) {
    skip_flags |= BLO_READ_SKIP_DATA;
  }
  if (use_userdef == false) {
    skip_flags |= BLO_READ_SKIP_USERDEF;
  }

  /* True if we load startup.blend from memory
   * or use app-template startup.blend which the user hasn't saved. */
  bool is_factory_startup = true;

  const char *app_template = nullptr;
  bool update_defaults = false;

  if (filepath_startup_override != nullptr) {
    /* pass */
  }
  else if (app_template_override) {
    /* This may be clearing the current template by setting to an empty string. */
    app_template = app_template_override;
  }
  else if (!use_factory_settings && U.app_template[0]) {
    app_template = U.app_template;
  }

  const bool reset_app_template = ((!app_template && U.app_template[0]) ||
                                   (app_template && !STREQ(app_template, U.app_template)));

  /* Options exclude each other. */
  BLI_assert((use_factory_settings && filepath_startup_override) == 0);

  if ((G.f & G_FLAG_SCRIPT_OVERRIDE_PREF) == 0) {
    SET_FLAG_FROM_TEST(G.f, (U.flag & USER_SCRIPT_AUTOEXEC_DISABLE) == 0, G_FLAG_SCRIPT_AUTOEXEC);
  }

  if (use_data) {
    if (reset_app_template) {
      /* Always load UI when switching to another template. */
      G.fileflags &= ~G_FILE_NO_UI;
    }
  }

  if (use_userdef || reset_app_template) {
#ifdef WITH_PYTHON
    /* This only runs once Blender has already started. */
    if (CTX_py_init_get(C)) {
      /* This is restored by 'wm_file_read_post', disable before loading any preferences
       * so an add-on can read their own preferences when un-registering,
       * and use new preferences if/when re-registering, see #67577.
       *
       * Note that this fits into 'wm_file_read_pre' function but gets messy
       * since we need to know if 'reset_app_template' is true. */
      const char *imports[] = {"addon_utils", nullptr};
      BPY_run_string_eval(C, imports, "addon_utils.disable_all()");
    }
#endif /* WITH_PYTHON */
  }

  if (use_data) {
    /* NOTE: either #BKE_CB_EVT_LOAD_POST or #BKE_CB_EVT_LOAD_POST_FAIL must run.
     * This runs from #wm_homefile_read_post. */
    BKE_callback_exec_string(CTX_data_main(C), BKE_CB_EVT_LOAD_PRE, "");
  }

  /* For regular file loading this only runs after the file is successfully read.
   * In the case of the startup file, the in-memory startup file is used as a fallback
   * so we know this will work if all else fails. */
  wm_file_read_pre(use_data, use_userdef);

  if (use_data) {
    /* put aside screens to match with persistent windows later */
    wm_window_match_init(C, &wmbase);
  }

  filepath_startup[0] = '\0';
  filepath_userdef[0] = '\0';
  app_template_system[0] = '\0';
  app_template_config[0] = '\0';

  const char *const cfgdir = BKE_appdir_folder_id(BLENDER_USER_CONFIG, nullptr);
  if (!use_factory_settings) {
    if (cfgdir) {
      BLI_path_join(filepath_startup, sizeof(filepath_startup), cfgdir, BLENDER_STARTUP_FILE);
      filepath_startup_is_factory = false;
      if (use_userdef) {
        BLI_path_join(filepath_userdef, sizeof(filepath_startup), cfgdir, BLENDER_USERPREF_FILE);
      }
    }
    else {
      use_factory_settings = true;
    }

    if (filepath_startup_override) {
      BLI_strncpy(filepath_startup, filepath_startup_override, FILE_MAX);
      filepath_startup_is_factory = false;
    }
  }

  /* load preferences before startup.blend */
  if (use_userdef) {
    if (use_factory_settings_app_template_only) {
      /* Use the current preferences as-is (only load in the app_template preferences). */
      skip_flags |= BLO_READ_SKIP_USERDEF;
    }
    else if (!use_factory_settings && BLI_exists(filepath_userdef)) {
      UserDef *userdef = BKE_blendfile_userdef_read(filepath_userdef, nullptr);
      if (userdef != nullptr) {
        BKE_blender_userdef_data_set_and_free(userdef);
        userdef = nullptr;

        skip_flags |= BLO_READ_SKIP_USERDEF;
        printf("Read prefs: \"%s\"\n", filepath_userdef);
      }
    }
  }

  if ((app_template != nullptr) && (app_template[0] != '\0')) {
    if (!BKE_appdir_app_template_id_search(
            app_template, app_template_system, sizeof(app_template_system))) {
      /* Can safely continue with code below, just warn it's not found. */
      BKE_reportf(reports, RPT_WARNING, "Application Template \"%s\" not found", app_template);
    }

    /* Insert template name into startup file. */

    /* note that the path is being set even when 'use_factory_settings == true'
     * this is done so we can load a templates factory-settings */
    if (!use_factory_settings) {
      BLI_path_join(app_template_config, sizeof(app_template_config), cfgdir, app_template);
      BLI_path_join(
          filepath_startup, sizeof(filepath_startup), app_template_config, BLENDER_STARTUP_FILE);
      filepath_startup_is_factory = false;
      if (BLI_access(filepath_startup, R_OK) != 0) {
        filepath_startup[0] = '\0';
      }
    }
    else {
      filepath_startup[0] = '\0';
    }

    if (filepath_startup[0] == '\0') {
      BLI_path_join(
          filepath_startup, sizeof(filepath_startup), app_template_system, BLENDER_STARTUP_FILE);
      filepath_startup_is_factory = true;

      /* Update defaults only for system templates. */
      update_defaults = true;
    }
  }

  if (!use_factory_settings || (filepath_startup[0] != '\0')) {
    if (BLI_access(filepath_startup, R_OK) == 0) {
      BlendFileReadParams params{};
      params.is_startup = true;
      params.skip_flags = skip_flags | BLO_READ_SKIP_USERDEF;
      BlendFileReadReport bf_reports{};
      bf_reports.reports = reports;
      struct BlendFileData *bfd = BKE_blendfile_read(filepath_startup, &params, &bf_reports);

      if (bfd != nullptr) {
        BKE_blendfile_read_setup_ex(
            C, bfd, &params, &bf_reports, update_defaults && use_data, app_template);
        success = true;
      }
    }
    if (success) {
      is_factory_startup = filepath_startup_is_factory;
    }
  }

  if (use_userdef) {
    if ((skip_flags & BLO_READ_SKIP_USERDEF) == 0) {
      UserDef *userdef_default = BKE_blendfile_userdef_from_defaults();
      BKE_blender_userdef_data_set_and_free(userdef_default);
      skip_flags |= BLO_READ_SKIP_USERDEF;
    }
  }

  if (success == false && filepath_startup_override && reports) {
    /* We can not return from here because wm is already reset */
    BKE_reportf(reports, RPT_ERROR, "Could not read \"%s\"", filepath_startup_override);
  }

  if (success == false) {
    BlendFileReadParams read_file_params{};
    read_file_params.is_startup = true;
    read_file_params.skip_flags = skip_flags;
    struct BlendFileData *bfd = BKE_blendfile_read_from_memory(
        datatoc_startup_blend, datatoc_startup_blend_size, &read_file_params, nullptr);
    if (bfd != nullptr) {
      BlendFileReadReport read_report{};
      BKE_blendfile_read_setup_ex(C, bfd, &read_file_params, &read_report, true, nullptr);
      success = true;
    }

    if (use_data && BLI_listbase_is_empty(&wmbase)) {
      wm_clear_default_size(C);
    }
  }

  if (use_empty_data) {
    BKE_blendfile_read_make_empty(C);
  }

  /* Load template preferences,
   * unlike regular preferences we only use some of the settings,
   * see: BKE_blender_userdef_set_app_template */
  if (app_template_system[0] != '\0') {
    char temp_path[FILE_MAX];
    temp_path[0] = '\0';
    if (!use_factory_settings) {
      BLI_path_join(temp_path, sizeof(temp_path), app_template_config, BLENDER_USERPREF_FILE);
      if (BLI_access(temp_path, R_OK) != 0) {
        temp_path[0] = '\0';
      }
    }

    if (temp_path[0] == '\0') {
      BLI_path_join(temp_path, sizeof(temp_path), app_template_system, BLENDER_USERPREF_FILE);
    }

    if (use_userdef) {
      UserDef *userdef_template = nullptr;
      /* just avoids missing file warning */
      if (BLI_exists(temp_path)) {
        userdef_template = BKE_blendfile_userdef_read(temp_path, nullptr);
      }
      if (userdef_template == nullptr) {
        /* we need to have preferences load to overwrite preferences from previous template */
        userdef_template = BKE_blendfile_userdef_from_defaults();
      }
      if (userdef_template) {
        BKE_blender_userdef_app_template_data_set_and_free(userdef_template);
        userdef_template = nullptr;
      }
    }
  }

  if (app_template_override) {
    BLI_strncpy(U.app_template, app_template_override, sizeof(U.app_template));
  }

  Main *bmain = CTX_data_main(C);

  if (use_userdef) {
    /* check userdef before open window, keymaps etc */
    wm_init_userdef(bmain);
  }

  if (use_data) {
    /* match the read WM with current WM */
    wm_window_match_do(C, &wmbase, &bmain->wm, &bmain->wm);
  }

  if (use_userdef) {
    /* Clear keymaps because the current default keymap may have been initialized
     * from user preferences, which have been reset. */
    LISTBASE_FOREACH (wmWindowManager *, wm, &bmain->wm) {
      if (wm->defaultconf) {
        wm->defaultconf->flag &= ~KEYCONF_INIT_DEFAULT;
      }
    }
  }

  if (use_data) {
    WM_check(C); /* opens window(s), checks keymaps */

    bmain->filepath[0] = '\0';
  }

  {
    wmFileReadPost_Params params_file_read_post{};
    params_file_read_post.use_data = use_data;
    params_file_read_post.use_userdef = use_userdef;
    params_file_read_post.is_startup_file = true;
    params_file_read_post.is_factory_startup = is_factory_startup;
    params_file_read_post.reset_app_template = reset_app_template;

    params_file_read_post.success = success;
    params_file_read_post.is_alloc = false;

    if (r_params_file_read_post == nullptr) {
      wm_homefile_read_post(C, &params_file_read_post);
    }
    else {
      params_file_read_post.is_alloc = true;
      *r_params_file_read_post = static_cast<wmFileReadPost_Params *>(
          MEM_mallocN(sizeof(wmFileReadPost_Params), __func__));
      **r_params_file_read_post = params_file_read_post;

      /* Match #wm_file_read_post which leaves the window cleared too. */
      CTX_wm_window_set(C, nullptr);
    }
  }
}

void wm_homefile_read(bContext *C,
                      const struct wmHomeFileRead_Params *params_homefile,
                      ReportList *reports)
{
  wm_homefile_read_ex(C, params_homefile, reports, nullptr);
}

void wm_homefile_read_post(struct bContext *C,
                           const struct wmFileReadPost_Params *params_file_read_post)
{
  wm_file_read_post(C, params_file_read_post);

  if (params_file_read_post->use_data) {
    BKE_callback_exec_string(CTX_data_main(C),
                             params_file_read_post->success ? BKE_CB_EVT_LOAD_POST :
                                                              BKE_CB_EVT_LOAD_POST_FAIL,
                             "");
  }

  if (params_file_read_post->is_alloc) {
    MEM_freeN((void *)params_file_read_post);
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Blend-File History API
 * \{ */

void wm_history_file_read(void)
{
  const char *const cfgdir = BKE_appdir_folder_id(BLENDER_USER_CONFIG, nullptr);
  if (!cfgdir) {
    return;
  }

  char name[FILE_MAX];
  LinkNode *l;
  int num;

  BLI_path_join(name, sizeof(name), cfgdir, BLENDER_HISTORY_FILE);

  LinkNode *lines = BLI_file_read_as_lines(name);

  wm_history_files_free();

  /* Read list of recent opened files from #BLENDER_HISTORY_FILE to memory. */
  for (l = lines, num = 0; l && (num < U.recent_files); l = l->next) {
    const char *line = static_cast<const char *>(l->link);
    /* don't check if files exist, causes slow startup for remote/external drives */
    if (line[0]) {
      struct RecentFile *recent = (RecentFile *)MEM_mallocN(sizeof(RecentFile), "RecentFile");
      BLI_addtail(&(G.recent_files), recent);
      recent->filepath = BLI_strdup(line);
      num++;
    }
  }

  BLI_file_free_lines(lines);
}

static RecentFile *wm_history_file_new(const char *filepath)
{
  RecentFile *recent = static_cast<RecentFile *>(MEM_mallocN(sizeof(RecentFile), "RecentFile"));
  recent->filepath = BLI_strdup(filepath);
  return recent;
}

static void wm_history_file_free(RecentFile *recent)
{
  BLI_assert(BLI_findindex(&G.recent_files, recent) != -1);
  MEM_freeN(recent->filepath);
  BLI_freelinkN(&G.recent_files, recent);
}

static void wm_history_files_free(void)
{
  LISTBASE_FOREACH_MUTABLE (RecentFile *, recent, &G.recent_files) {
    wm_history_file_free(recent);
  }
}

static RecentFile *wm_file_history_find(const char *filepath)
{
  return static_cast<RecentFile *>(
      BLI_findstring_ptr(&G.recent_files, filepath, offsetof(RecentFile, filepath)));
}

/**
 * Write #BLENDER_HISTORY_FILE as-is, without checking the environment
 * (that's handled by #wm_history_file_update).
 */
static void wm_history_file_write(void)
{
  const char *user_config_dir;
  char name[FILE_MAX];
  FILE *fp;

  /* will be nullptr in background mode */
  user_config_dir = BKE_appdir_folder_id_create(BLENDER_USER_CONFIG, nullptr);
  if (!user_config_dir) {
    return;
  }

  BLI_path_join(name, sizeof(name), user_config_dir, BLENDER_HISTORY_FILE);

  fp = BLI_fopen(name, "w");
  if (fp) {
    LISTBASE_FOREACH (RecentFile *, recent, &G.recent_files) {
      fprintf(fp, "%s\n", recent->filepath);
    }
    fclose(fp);
  }
}

/**
 * Run after saving a file to refresh the #BLENDER_HISTORY_FILE list.
 */
static void wm_history_file_update(void)
{
  RecentFile *recent;
  const char *blendfile_path = BKE_main_blendfile_path_from_global();

  /* No write history for recovered startup files. */
  if (blendfile_path[0] == '\0') {
    return;
  }

  recent = static_cast<RecentFile *>(G.recent_files.first);
  /* Refresh #BLENDER_HISTORY_FILE of recent opened files, when current file was changed. */
  if (!(recent) || (BLI_path_cmp(recent->filepath, blendfile_path) != 0)) {

    recent = wm_file_history_find(blendfile_path);
    if (recent) {
      BLI_remlink(&G.recent_files, recent);
    }
    else {
      RecentFile *recent_next;
      for (recent = static_cast<RecentFile *>(BLI_findlink(&G.recent_files, U.recent_files - 1));
           recent;
           recent = recent_next) {
        recent_next = recent->next;
        wm_history_file_free(recent);
      }
      recent = wm_history_file_new(blendfile_path);
    }

    /* add current file to the beginning of list */
    BLI_addhead(&(G.recent_files), recent);

    /* Write current file to #BLENDER_HISTORY_FILE. */
    wm_history_file_write();

    /* also update most recent files on System */
    GHOST_addToSystemRecentFiles(blendfile_path);
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Thumbnail Generation: Screen-Shot / Camera View
 *
 * Thumbnail Sizes
 * ===============
 *
 * - `PREVIEW_RENDER_LARGE_HEIGHT * 2` is used to render a large thumbnail,
 *   giving some over-sampling when scaled down:
 *
 * - There are two outputs for this thumbnail:
 *
 *   - An image is saved to the thumbnail cache, sized at #PREVIEW_RENDER_LARGE_HEIGHT.
 *
 *   - A smaller thumbnail is stored in the `.blend` file itself, sized at #BLEN_THUMB_SIZE.
 *     The size is kept small to prevent thumbnails bloating the size of `.blend` files.
 *
 *     The this thumbnail will be extracted if the file is shared or the local thumbnail cache
 *     is cleared. see: `blendthumb_extract.cc` for logic that extracts the thumbnail.
 *
 * \{ */

/**
 * Screen-shot the active window.
 */
static ImBuf *blend_file_thumb_from_screenshot(bContext *C, BlendThumbnail **r_thumb)
{
  *r_thumb = nullptr;

  wmWindow *win = CTX_wm_window(C);
  if (G.background || (win == nullptr)) {
    return nullptr;
  }

  /* The window to capture should be a main window (without parent). */
  while (win && win->parent) {
    win = win->parent;
  }

  int win_size[2];
  uint *buffer = WM_window_pixels_read(CTX_wm_manager(C), win, win_size);
  ImBuf *ibuf = IMB_allocFromBufferOwn(buffer, nullptr, win_size[0], win_size[1], 24);

  if (ibuf) {
    int ex, ey;
    if (ibuf->x > ibuf->y) {
      ex = BLEN_THUMB_SIZE;
      ey = max_ii(1, int((float(ibuf->y) / float(ibuf->x)) * BLEN_THUMB_SIZE));
    }
    else {
      ex = max_ii(1, int((float(ibuf->x) / float(ibuf->y)) * BLEN_THUMB_SIZE));
      ey = BLEN_THUMB_SIZE;
    }

    /* File-system thumbnail image can be 256x256. */
    IMB_scaleImBuf(ibuf, ex * 2, ey * 2);

    /* Thumbnail inside blend should be 128x128. */
    ImBuf *thumb_ibuf = IMB_dupImBuf(ibuf);
    IMB_scaleImBuf(thumb_ibuf, ex, ey);

    BlendThumbnail *thumb = BKE_main_thumbnail_from_imbuf(nullptr, thumb_ibuf);
    IMB_freeImBuf(thumb_ibuf);
    *r_thumb = thumb;
  }

  /* Must be freed by caller. */
  return ibuf;
}

/**
 * Render the current scene with the active camera.
 *
 * \param screen: can be nullptr.
 */
static ImBuf *blend_file_thumb_from_camera(const bContext *C,
                                           Scene *scene,
                                           bScreen *screen,
                                           BlendThumbnail **r_thumb)
{
  *r_thumb = nullptr;

  /* Scene can be nullptr if running a script at startup and calling the save operator. */
  if (G.background || scene == nullptr) {
    return nullptr;
  }

  /* will be scaled down, but gives some nice oversampling */
  ImBuf *ibuf;
  BlendThumbnail *thumb;
  wmWindowManager *wm = CTX_wm_manager(C);
  const float pixelsize_old = U.pixelsize;
  wmWindow *windrawable_old = wm->windrawable;
  char err_out[256] = "unknown";

  /* screen if no camera found */
  ScrArea *area = nullptr;
  ARegion *region = nullptr;
  View3D *v3d = nullptr;

  if (screen != nullptr) {
    area = BKE_screen_find_big_area(screen, SPACE_VIEW3D, 0);
    if (area) {
      v3d = static_cast<View3D *>(area->spacedata.first);
      region = BKE_area_find_region_type(area, RGN_TYPE_WINDOW);
    }
  }

  if (scene->camera == nullptr && v3d == nullptr) {
    return nullptr;
  }

  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);

  /* Note that with scaling, this ends up being 0.5,
   * as it's a thumbnail, we don't need object centers and friends to be 1:1 size. */
  U.pixelsize = 1.0f;

  if (scene->camera) {
    ibuf = ED_view3d_draw_offscreen_imbuf_simple(depsgraph,
                                                 scene,
                                                 (v3d) ? &v3d->shading : nullptr,
                                                 (v3d) ? eDrawType(v3d->shading.type) : OB_SOLID,
                                                 scene->camera,
                                                 PREVIEW_RENDER_LARGE_HEIGHT * 2,
                                                 PREVIEW_RENDER_LARGE_HEIGHT * 2,
                                                 IB_rect,
                                                 (v3d) ? V3D_OFSDRAW_OVERRIDE_SCENE_SETTINGS :
                                                         V3D_OFSDRAW_NONE,
                                                 R_ALPHAPREMUL,
                                                 nullptr,
                                                 nullptr,
                                                 err_out);
  }
  else {
    ibuf = ED_view3d_draw_offscreen_imbuf(depsgraph,
                                          scene,
                                          OB_SOLID,
                                          v3d,
                                          region,
                                          PREVIEW_RENDER_LARGE_HEIGHT * 2,
                                          PREVIEW_RENDER_LARGE_HEIGHT * 2,
                                          IB_rect,
                                          R_ALPHAPREMUL,
                                          nullptr,
                                          true,
                                          nullptr,
                                          err_out);
  }

  U.pixelsize = pixelsize_old;

  /* Reset to old drawable. */
  if (windrawable_old) {
    wm_window_make_drawable(wm, windrawable_old);
  }
  else {
    wm_window_clear_drawable(wm);
  }

  if (ibuf) {
    /* dirty oversampling */
    ImBuf *thumb_ibuf;
    thumb_ibuf = IMB_dupImBuf(ibuf);
    /* BLEN_THUMB_SIZE is size of thumbnail inside blend file: 128x128. */
    IMB_scaleImBuf(thumb_ibuf, BLEN_THUMB_SIZE, BLEN_THUMB_SIZE);
    thumb = BKE_main_thumbnail_from_imbuf(nullptr, thumb_ibuf);
    IMB_freeImBuf(thumb_ibuf);
    /* Thumbnail saved to file-system should be 256x256. */
    IMB_scaleImBuf(ibuf, PREVIEW_RENDER_LARGE_HEIGHT, PREVIEW_RENDER_LARGE_HEIGHT);
  }
  else {
    /* '*r_thumb' needs to stay nullptr to prevent a bad thumbnail from being handled. */
    CLOG_WARN(&LOG, "failed to create thumbnail: %s", err_out);
    thumb = nullptr;
  }

  /* must be freed by caller */
  *r_thumb = thumb;

  return ibuf;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Write Main Blend-File (internal)
 * \{ */

bool write_crash_blend(void)
{
  char filepath[FILE_MAX];

  BLI_strncpy(filepath, BKE_main_blendfile_path_from_global(), sizeof(filepath));
  BLI_path_extension_replace(filepath, sizeof(filepath), "_crash.blend");
  BlendFileWriteParams params{};
  const bool success = BLO_write_file(G_MAIN, filepath, G.fileflags, &params, nullptr);
  printf("%s: \"%s\"\n", success ? "written" : "failed", filepath);
  return success;
}

/**
 * Helper to check if file `filepath` can be written.
 * \return true if it can, otherwise report an error and return false.
 */
static bool wm_file_write_check_with_report_on_failure(Main *bmain,
                                                       const char *filepath,
                                                       ReportList *reports)
{
  const int filepath_len = strlen(filepath);
  if (filepath_len == 0) {
    BKE_report(reports, RPT_ERROR, "Path is empty, cannot save");
    return false;
  }

  if (filepath_len >= FILE_MAX) {
    BKE_report(reports, RPT_ERROR, "Path too long, cannot save");
    return false;
  }

  /* Check if file write permission is ok */
  if (BLI_exists(filepath) && !BLI_file_is_writable(filepath)) {
    BKE_reportf(
        reports, RPT_ERROR, "Cannot save blend file, path \"%s\" is not writable", filepath);
    return false;
  }

  LISTBASE_FOREACH (Library *, li, &bmain->libraries) {
    if (BLI_path_cmp(li->filepath_abs, filepath) == 0) {
      BKE_reportf(reports, RPT_ERROR, "Cannot overwrite used library '%.240s'", filepath);
      return false;
    }
  }

  return true;
}

/**
 * \see #wm_homefile_write_exec wraps #BLO_write_file in a similar way.
 */
static bool wm_file_write(bContext *C,
                          const char *filepath,
                          int fileflags,
                          eBLO_WritePathRemap remap_mode,
                          bool use_save_as_copy,
                          ReportList *reports)
{
  Main *bmain = CTX_data_main(C);
  BlendThumbnail *thumb = nullptr, *main_thumb = nullptr;
  ImBuf *ibuf_thumb = nullptr;

  /* NOTE: used to replace the file extension (to ensure '.blend'),
   * no need to now because the operator ensures,
   * its handy for scripts to save to a predefined name without blender editing it */

  if (!wm_file_write_check_with_report_on_failure(bmain, filepath, reports)) {
    return false;
  }

  /* Call pre-save callbacks before writing preview,
   * that way you can generate custom file thumbnail. */

  /* NOTE: either #BKE_CB_EVT_SAVE_POST or #BKE_CB_EVT_SAVE_POST_FAIL must run.
   * Runs at the end of this function, don't return beforehand. */
  BKE_callback_exec_string(bmain, BKE_CB_EVT_SAVE_PRE, filepath);
  ED_assets_pre_save(bmain);

  /* Enforce full override check/generation on file save. */
  BKE_lib_override_library_main_operations_create(bmain, true, nullptr);

  /* NOTE: Ideally we would call `WM_redraw_windows` here to remove any open menus.
   * But we can crash if saving from a script, see #92704 & #97627.
   * Just checking `!G.background && BLI_thread_is_main()` is not sufficient to fix this.
   * Additionally some EGL configurations don't support reading the front-buffer
   * immediately after drawing, see: #98462. In that case off-screen drawing is necessary. */

  /* don't forget not to return without! */
  WM_cursor_wait(true);

  if (U.file_preview_type != USER_FILE_PREVIEW_NONE) {
    /* Blend file thumbnail.
     *
     * - Save before exiting edit-mode, otherwise evaluated-mesh for shared data gets corrupted.
     *   See #27765.
     * - Main can store a '.blend' thumbnail,
     *   useful for background-mode or thumbnail customization.
     */
    main_thumb = thumb = bmain->blen_thumb;
    if (thumb != nullptr) {
      /* In case we are given a valid thumbnail data, just generate image from it. */
      ibuf_thumb = BKE_main_thumbnail_to_imbuf(nullptr, thumb);
    }
    else if (BLI_thread_is_main()) {
      int file_preview_type = U.file_preview_type;

      if (file_preview_type == USER_FILE_PREVIEW_AUTO) {
        Scene *scene = CTX_data_scene(C);
        bScreen *screen = CTX_wm_screen(C);
        bool do_render = (scene != nullptr && scene->camera != nullptr && screen != nullptr &&
                          (BKE_screen_find_big_area(screen, SPACE_VIEW3D, 0) != nullptr));
        file_preview_type = do_render ? USER_FILE_PREVIEW_CAMERA : USER_FILE_PREVIEW_SCREENSHOT;
      }

      switch (file_preview_type) {
        case USER_FILE_PREVIEW_SCREENSHOT: {
          ibuf_thumb = blend_file_thumb_from_screenshot(C, &thumb);
          break;
        }
        case USER_FILE_PREVIEW_CAMERA: {
          ibuf_thumb = blend_file_thumb_from_camera(
              C, CTX_data_scene(C), CTX_wm_screen(C), &thumb);
          break;
        }
        default:
          BLI_assert_unreachable();
      }
    }
  }

  /* operator now handles overwrite checks */

  if (G.fileflags & G_FILE_AUTOPACK) {
    BKE_packedfile_pack_all(bmain, reports, false);
  }

  ED_editors_flush_edits(bmain);

  /* XXX(ton): temp solution to solve bug, real fix coming. */
  bmain->recovered = false;

  BlendFileWriteParams blend_write_params{};
  blend_write_params.remap_mode = remap_mode;
  blend_write_params.use_save_versions = true;
  blend_write_params.use_save_as_copy = use_save_as_copy;
  blend_write_params.thumb = thumb;

  const bool success = BLO_write_file(bmain, filepath, fileflags, &blend_write_params, reports);

  if (success) {
    const bool do_history_file_update = (G.background == false) &&
                                        (CTX_wm_manager(C)->op_undo_depth == 0);

    if (use_save_as_copy == false) {
      STRNCPY(bmain->filepath, filepath); /* is guaranteed current file */
    }

    SET_FLAG_FROM_TEST(G.fileflags, fileflags & G_FILE_COMPRESS, G_FILE_COMPRESS);

    /* prevent background mode scripts from clobbering history */
    if (do_history_file_update) {
      wm_history_file_update();
    }

    /* run this function after because the file can't be written before the blend is */
    if (ibuf_thumb) {
      IMB_thumb_delete(filepath, THB_FAIL); /* without this a failed thumb overrides */
      ibuf_thumb = IMB_thumb_create(filepath, THB_LARGE, THB_SOURCE_BLEND, ibuf_thumb);
    }

    /* Without this there is no feedback the file was saved. */
    BKE_reportf(reports, RPT_INFO, "Saved \"%s\"", BLI_path_basename(filepath));
  }

  BKE_callback_exec_string(
      bmain, success ? BKE_CB_EVT_SAVE_POST : BKE_CB_EVT_SAVE_POST_FAIL, filepath);

  if (ibuf_thumb) {
    IMB_freeImBuf(ibuf_thumb);
  }
  if (thumb && thumb != main_thumb) {
    MEM_freeN(thumb);
  }

  WM_cursor_wait(false);

  return success;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Auto-Save API
 * \{ */

static void wm_autosave_location(char filepath[FILE_MAX])
{
  const int pid = abs(getpid());
  char filename[1024];

  /* Normally there is no need to check for this to be nullptr,
   * however this runs on exit when it may be cleared. */
  Main *bmain = G_MAIN;
  const char *blendfile_path = bmain ? BKE_main_blendfile_path(bmain) : nullptr;

  if (blendfile_path && (blendfile_path[0] != '\0')) {
    const char *basename = BLI_path_basename(blendfile_path);
    int len = strlen(basename) - 6;
    BLI_snprintf(filename, sizeof(filename), "%.*s_%d_autosave.blend", len, basename, pid);
  }
  else {
    BLI_snprintf(filename, sizeof(filename), "%d_autosave.blend", pid);
  }

  const char *tempdir_base = BKE_tempdir_base();
  /* NOTE(@ideasman42): It's strange that this is only used on WIN32.
   * From reading commits it seems accessing the temporary directory used to be less reliable.
   * If this is still the case on WIN32 - other features such as copy-paste will also fail.
   * We could support #BLENDER_USER_AUTOSAVE on all platforms or remove it entirely. */
#ifdef WIN32
  if (!BLI_exists(tempdir_base)) {
    const char *savedir = BKE_appdir_folder_id_create(BLENDER_USER_AUTOSAVE, nullptr);
    if (savedir) {
      tempdir_base = savedir;
    }
  }
#endif

  BLI_path_join(filepath, FILE_MAX, tempdir_base, filename);
}

/* TODO: Move to appropriate headers */
void ED_sculpt_fast_save_bmesh(Object *ob);
struct MemFileUndoStep;
MemFileUndoData *memfile_get_step_data(struct MemFileUndoStep *us);

extern "C" void wm_autosave_write(Main *bmain, wmWindowManager *wm)
{
  char filepath[FILE_MAX];

  wm_autosave_location(filepath);

  /* Fast save of last undo-buffer, now with UI. */
  const bool use_memfile = (U.uiflag & USER_GLOBALUNDO) != 0;
  MemFile *memfile = use_memfile ? ED_undosys_stack_memfile_get_active(wm->undo_stack) : nullptr;
  bool update = false;

  LISTBASE_FOREACH (Object *, ob, &bmain->objects) {
    if (ob->mode != OB_MODE_SCULPT || !ob->sculpt) {
      continue;
    }

    /* Flush sculpt data to the mesh, we will append it to the undo memfile. */
    if (ob->sculpt->bm) {
      ED_sculpt_fast_save_bmesh(ob);
    }
    else {
      multires_flush_sculpt_updates(ob);
    }

    update = true;
  }

  MemFileUndoData *mus = NULL;

  if (update && memfile) {
    UndoStep *us = BKE_undosys_stack_active_with_type(wm->undo_stack, BKE_UNDOSYS_TYPE_MEMFILE);

    if (us) {
      mus = memfile_get_step_data((struct MemFileUndoStep *)us);
      mus = BKE_memfile_undo_encode(bmain, mus);
      memfile = &mus->memfile;
    }
  }
  if (memfile != nullptr) {
    BLO_memfile_write_file(memfile, filepath);

    if (update) {
      BKE_memfile_undo_free(mus);
    }
  }
  else {
    if (use_memfile) {
      /* This is very unlikely, alert developers of this unexpected case. */
      CLOG_WARN(&LOG, "undo-data not found for writing, fallback to regular file write!");
    }

    /* Save as regular blend file with recovery information. */
    const int fileflags = (G.fileflags & ~G_FILE_COMPRESS) | G_FILE_RECOVER_WRITE;

    ED_editors_flush_edits(bmain);

    /* Error reporting into console. */
    BlendFileWriteParams params{};
    BLO_write_file(bmain, filepath, fileflags, &params, nullptr);
  }
}

static void wm_autosave_timer_begin_ex(wmWindowManager *wm, double timestep)
{
  wm_autosave_timer_end(wm);

  if (U.flag & USER_AUTOSAVE) {
    wm->autosavetimer = WM_event_add_timer(wm, nullptr, TIMERAUTOSAVE, timestep);
  }
}

void wm_autosave_timer_begin(wmWindowManager *wm)
{
  wm_autosave_timer_begin_ex(wm, U.savetime * 60.0);
}

void wm_autosave_timer_end(wmWindowManager *wm)
{
  if (wm->autosavetimer) {
    WM_event_remove_timer(wm, nullptr, wm->autosavetimer);
    wm->autosavetimer = nullptr;
  }
}

void WM_file_autosave_init(wmWindowManager *wm)
{
  wm_autosave_timer_begin(wm);
}

void wm_autosave_timer(Main *bmain, wmWindowManager *wm, wmTimer * /*wt*/)
{
  wm_autosave_timer_end(wm);

  /* If a modal operator is running, don't autosave because we might not be in
   * a valid state to save. But try again in 10ms. */
  LISTBASE_FOREACH (wmWindow *, win, &wm->windows) {
    LISTBASE_FOREACH (wmEventHandler *, handler_base, &win->modalhandlers) {
      if (handler_base->type == WM_HANDLER_TYPE_OP) {
        wmEventHandler_Op *handler = (wmEventHandler_Op *)handler_base;
        if (handler->op) {
          wm_autosave_timer_begin_ex(wm, 0.01);
          return;
        }
      }
    }
  }

  wm_autosave_write(bmain, wm);

  /* Restart the timer after file write, just in case file write takes a long time. */
  wm_autosave_timer_begin(wm);
}

void wm_autosave_delete(void)
{
  char filepath[FILE_MAX];

  wm_autosave_location(filepath);

  if (BLI_exists(filepath)) {
    char str[FILE_MAX];
    BLI_path_join(str, sizeof(str), BKE_tempdir_base(), BLENDER_QUIT_FILE);

    /* For global undo; remove temporarily saved file, otherwise rename. */
    if (U.uiflag & USER_GLOBALUNDO) {
      BLI_delete(filepath, false, false);
    }
    else {
      BLI_rename(filepath, str);
    }
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Shared Operator Properties
 * \{ */

/** Use for loading factory startup & preferences. */
static void read_factory_reset_props(wmOperatorType *ot)
{
  PropertyRNA *prop;

  /* So it's possible to reset app-template settings without resetting other defaults. */
  prop = RNA_def_boolean(ot->srna,
                         "use_factory_startup_app_template_only",
                         false,
                         "Factory Startup App-Template Only",
                         "");
  RNA_def_property_flag(prop, PropertyFlag(PROP_HIDDEN | PROP_SKIP_SAVE));
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Initialize `WM_OT_open_*` Properties
 *
 * Check if load_ui was set by the caller.
 * Fall back to user preference when file flags not specified.
 *
 * \{ */

void wm_open_init_load_ui(wmOperator *op, bool use_prefs)
{
  PropertyRNA *prop = RNA_struct_find_property(op->ptr, "load_ui");
  if (!RNA_property_is_set(op->ptr, prop)) {
    bool value = use_prefs ? ((U.flag & USER_FILENOUI) == 0) : ((G.fileflags & G_FILE_NO_UI) == 0);

    RNA_property_boolean_set(op->ptr, prop, value);
  }
}

void wm_open_init_use_scripts(wmOperator *op, bool use_prefs)
{
  PropertyRNA *prop = RNA_struct_find_property(op->ptr, "use_scripts");
  if (!RNA_property_is_set(op->ptr, prop)) {
    /* use G_FLAG_SCRIPT_AUTOEXEC rather than the userpref because this means if
     * the flag has been disabled from the command line, then opening
     * from the menu won't enable this setting. */
    bool value = use_prefs ? ((U.flag & USER_SCRIPT_AUTOEXEC_DISABLE) == 0) :
                             ((G.f & G_FLAG_SCRIPT_AUTOEXEC) != 0);

    RNA_property_boolean_set(op->ptr, prop, value);
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Startup File Save Operator
 * \{ */

/**
 * \see #wm_file_write wraps #BLO_write_file in a similar way.
 * \return success.
 */
static int wm_homefile_write_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  wmWindowManager *wm = CTX_wm_manager(C);
  wmWindow *win = CTX_wm_window(C);
  char filepath[FILE_MAX];
  int fileflags;

  const char *app_template = U.app_template[0] ? U.app_template : nullptr;
  const char *const cfgdir = BKE_appdir_folder_id_create(BLENDER_USER_CONFIG, app_template);
  if (cfgdir == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Unable to create user config path");
    return OPERATOR_CANCELLED;
  }

  /* NOTE: either #BKE_CB_EVT_SAVE_POST or #BKE_CB_EVT_SAVE_POST_FAIL must run.
   * Runs at the end of this function, don't return beforehand. */
  BKE_callback_exec_string(bmain, BKE_CB_EVT_SAVE_PRE, "");
  ED_assets_pre_save(bmain);

  /* check current window and close it if temp */
  if (win && WM_window_is_temp_screen(win)) {
    wm_window_close(C, wm, win);
  }

  /* update keymaps in user preferences */
  WM_keyconfig_update(wm);

  BLI_path_join(filepath, sizeof(filepath), cfgdir, BLENDER_STARTUP_FILE);

  printf("Writing homefile: \"%s\" ", filepath);

  ED_editors_flush_edits(bmain);

  /* Force save as regular blend file. */
  fileflags = G.fileflags & ~G_FILE_COMPRESS;

  BlendFileWriteParams blend_write_params{};
  /* Make all paths absolute when saving the startup file.
   * On load the `G.main->filepath` will be empty so the paths
   * won't have a base for resolving the relative paths. */
  blend_write_params.remap_mode = BLO_WRITE_PATH_REMAP_ABSOLUTE;
  /* Don't apply any path changes to the current blend file. */
  blend_write_params.use_save_as_copy = true;

  const bool success = BLO_write_file(
      bmain, filepath, fileflags, &blend_write_params, op->reports);

  BKE_callback_exec_string(bmain, success ? BKE_CB_EVT_SAVE_POST : BKE_CB_EVT_SAVE_POST_FAIL, "");

  if (success) {
    printf("ok\n");
    BKE_report(op->reports, RPT_INFO, "Startup file saved");
    return OPERATOR_FINISHED;
  }
  printf("fail\n");
  return OPERATOR_CANCELLED;
}

void WM_OT_save_homefile(wmOperatorType *ot)
{
  ot->name = "Save Startup File";
  ot->idname = "WM_OT_save_homefile";
  ot->description = "Make the current file the default .blend file";

  ot->invoke = WM_operator_confirm;
  ot->exec = wm_homefile_write_exec;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Write Preferences Operator
 * \{ */

/* Only save the prefs block. operator entry */
static int wm_userpref_write_exec(bContext *C, wmOperator *op)
{
  wmWindowManager *wm = CTX_wm_manager(C);

  /* Update keymaps in user preferences. */
  WM_keyconfig_update(wm);

  const bool success = BKE_blendfile_userdef_write_all(op->reports);

  return success ? OPERATOR_FINISHED : OPERATOR_CANCELLED;
}

void WM_OT_save_userpref(wmOperatorType *ot)
{
  ot->name = "Save Preferences";
  ot->idname = "WM_OT_save_userpref";
  ot->description = "Make the current preferences default";

  ot->invoke = WM_operator_confirm;
  ot->exec = wm_userpref_write_exec;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Read Preferences Operator
 * \{ */

/**
 * When reading preferences, there are some exceptions for values which are reset.
 */
static void wm_userpref_read_exceptions(UserDef *userdef_curr, const UserDef *userdef_prev)
{
#define USERDEF_RESTORE(member) \
  { \
    userdef_curr->member = userdef_prev->member; \
  } \
  ((void)0)

  /* Current visible preferences category. */
  USERDEF_RESTORE(space_data.section_active);

#undef USERDEF_RESTORE
}

static void rna_struct_update_when_changed(bContext *C,
                                           Main *bmain,
                                           PointerRNA *ptr_a,
                                           PointerRNA *ptr_b)
{
  CollectionPropertyIterator iter;
  PropertyRNA *iterprop = RNA_struct_iterator_property(ptr_a->type);
  BLI_assert(ptr_a->type == ptr_b->type);
  RNA_property_collection_begin(ptr_a, iterprop, &iter);
  for (; iter.valid; RNA_property_collection_next(&iter)) {
    PropertyRNA *prop = static_cast<PropertyRNA *>(iter.ptr.data);
    if (STREQ(RNA_property_identifier(prop), "rna_type")) {
      continue;
    }
    switch (RNA_property_type(prop)) {
      case PROP_POINTER: {
        PointerRNA ptr_sub_a = RNA_property_pointer_get(ptr_a, prop);
        PointerRNA ptr_sub_b = RNA_property_pointer_get(ptr_b, prop);
        rna_struct_update_when_changed(C, bmain, &ptr_sub_a, &ptr_sub_b);
        break;
      }
      case PROP_COLLECTION:
        /* Don't handle collections. */
        break;
      default: {
        if (!RNA_property_equals(bmain, ptr_a, ptr_b, prop, RNA_EQ_STRICT)) {
          RNA_property_update(C, ptr_b, prop);
        }
      }
    }
  }
  RNA_property_collection_end(&iter);
}

static void wm_userpref_update_when_changed(bContext *C,
                                            Main *bmain,
                                            UserDef *userdef_prev,
                                            UserDef *userdef_curr)
{
  PointerRNA ptr_a, ptr_b;
  RNA_pointer_create(nullptr, &RNA_Preferences, userdef_prev, &ptr_a);
  RNA_pointer_create(nullptr, &RNA_Preferences, userdef_curr, &ptr_b);
  const bool is_dirty = userdef_curr->runtime.is_dirty;

  rna_struct_update_when_changed(C, bmain, &ptr_a, &ptr_b);

  WM_reinit_gizmomap_all(bmain);
  WM_keyconfig_reload(C);

  userdef_curr->runtime.is_dirty = is_dirty;
}

static int wm_userpref_read_exec(bContext *C, wmOperator *op)
{
  const bool use_data = false;
  const bool use_userdef = true;
  const bool use_factory_settings = STREQ(op->type->idname, "WM_OT_read_factory_userpref");
  const bool use_factory_settings_app_template_only =
      (use_factory_settings && RNA_boolean_get(op->ptr, "use_factory_startup_app_template_only"));

  UserDef U_backup = blender::dna::shallow_copy(U);

  wmHomeFileRead_Params read_homefile_params{};
  read_homefile_params.use_data = use_data;
  read_homefile_params.use_userdef = use_userdef;
  read_homefile_params.use_factory_settings = use_factory_settings;
  read_homefile_params.use_factory_settings_app_template_only =
      use_factory_settings_app_template_only;
  read_homefile_params.use_empty_data = false;
  read_homefile_params.filepath_startup_override = nullptr;
  read_homefile_params.app_template_override = WM_init_state_app_template_get();
  wm_homefile_read(C, &read_homefile_params, op->reports);

  wm_userpref_read_exceptions(&U, &U_backup);
  SET_FLAG_FROM_TEST(G.f, use_factory_settings, G_FLAG_USERPREF_NO_SAVE_ON_EXIT);

  Main *bmain = CTX_data_main(C);

  wm_userpref_update_when_changed(C, bmain, &U_backup, &U);

  if (use_factory_settings) {
    U.runtime.is_dirty = true;
  }

  /* Needed to recalculate UI scaling values (eg, #UserDef.inv_dpi_fac). */
  wm_window_clear_drawable(static_cast<wmWindowManager *>(bmain->wm.first));

  WM_event_add_notifier(C, NC_WINDOW, nullptr);

  return OPERATOR_FINISHED;
}

void WM_OT_read_userpref(wmOperatorType *ot)
{
  ot->name = "Load Preferences";
  ot->idname = "WM_OT_read_userpref";
  ot->description = "Load last saved preferences";

  ot->invoke = WM_operator_confirm;
  ot->exec = wm_userpref_read_exec;
}

void WM_OT_read_factory_userpref(wmOperatorType *ot)
{
  ot->name = "Load Factory Preferences";
  ot->idname = "WM_OT_read_factory_userpref";
  ot->description =
      "Load factory default preferences. "
      "To make changes to preferences permanent, use \"Save Preferences\"";

  ot->invoke = WM_operator_confirm;
  ot->exec = wm_userpref_read_exec;

  read_factory_reset_props(ot);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Read File History Operator
 * \{ */

static int wm_history_file_read_exec(bContext * /*C*/, wmOperator * /*op*/)
{
  ED_file_read_bookmarks();
  wm_history_file_read();
  return OPERATOR_FINISHED;
}

void WM_OT_read_history(wmOperatorType *ot)
{
  ot->name = "Reload History File";
  ot->idname = "WM_OT_read_history";
  ot->description = "Reloads history and bookmarks";

  ot->invoke = WM_operator_confirm;
  ot->exec = wm_history_file_read_exec;

  /* this operator is only used for loading settings from a previous blender install */
  ot->flag = OPTYPE_INTERNAL;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Read Startup & Preferences Operator
 *
 * Both #WM_OT_read_homefile & #WM_OT_read_factory_settings.
 * \{ */

static int wm_homefile_read_exec(bContext *C, wmOperator *op)
{
  const bool use_factory_startup_and_userdef = STREQ(op->type->idname,
                                                     "WM_OT_read_factory_settings");
  const bool use_factory_settings = use_factory_startup_and_userdef ||
                                    RNA_boolean_get(op->ptr, "use_factory_startup");
  const bool use_factory_settings_app_template_only =
      (use_factory_startup_and_userdef &&
       RNA_boolean_get(op->ptr, "use_factory_startup_app_template_only"));

  bool use_userdef = false;
  char filepath_buf[FILE_MAX];
  const char *filepath = nullptr;
  UserDef U_backup = blender::dna::shallow_copy(U);

  if (!use_factory_settings) {
    PropertyRNA *prop = RNA_struct_find_property(op->ptr, "filepath");

    /* This can be used when loading of a start-up file should only change
     * the scene content but keep the blender UI as it is. */
    wm_open_init_load_ui(op, true);
    SET_FLAG_FROM_TEST(G.fileflags, !RNA_boolean_get(op->ptr, "load_ui"), G_FILE_NO_UI);

    if (RNA_property_is_set(op->ptr, prop)) {
      RNA_property_string_get(op->ptr, prop, filepath_buf);
      filepath = filepath_buf;
      if (BLI_access(filepath, R_OK)) {
        BKE_reportf(
            op->reports, RPT_ERROR, "Can't read alternative start-up file: \"%s\"", filepath);
        return OPERATOR_CANCELLED;
      }
    }
  }
  else {
    if (use_factory_startup_and_userdef) {
      /* always load UI for factory settings (prefs will re-init) */
      G.fileflags &= ~G_FILE_NO_UI;
      /* Always load preferences with factory settings. */
      use_userdef = true;
    }
  }

  char app_template_buf[sizeof(U.app_template)];
  const char *app_template;
  PropertyRNA *prop_app_template = RNA_struct_find_property(op->ptr, "app_template");
  const bool use_splash = !use_factory_settings && RNA_boolean_get(op->ptr, "use_splash");
  const bool use_empty_data = RNA_boolean_get(op->ptr, "use_empty");

  if (prop_app_template && RNA_property_is_set(op->ptr, prop_app_template)) {
    RNA_property_string_get(op->ptr, prop_app_template, app_template_buf);
    app_template = app_template_buf;

    if (!use_factory_settings) {
      /* Always load preferences when switching templates with own preferences. */
      use_userdef = BKE_appdir_app_template_has_userpref(app_template) ||
                    BKE_appdir_app_template_has_userpref(U.app_template);
    }

    /* Turn override off, since we're explicitly loading a different app-template. */
    WM_init_state_app_template_set(nullptr);
  }
  else {
    /* Normally nullptr, only set when overriding from the command-line. */
    app_template = WM_init_state_app_template_get();
  }

  wmHomeFileRead_Params read_homefile_params{};
  read_homefile_params.use_data = true;
  read_homefile_params.use_userdef = use_userdef;
  read_homefile_params.use_factory_settings = use_factory_settings;
  read_homefile_params.use_factory_settings_app_template_only =
      use_factory_settings_app_template_only;
  read_homefile_params.use_empty_data = use_empty_data;
  read_homefile_params.filepath_startup_override = filepath;
  read_homefile_params.app_template_override = app_template;
  wm_homefile_read(C, &read_homefile_params, op->reports);

  if (use_splash) {
    WM_init_splash(C);
  }

  if (use_userdef) {
    wm_userpref_read_exceptions(&U, &U_backup);
    SET_FLAG_FROM_TEST(G.f, use_factory_settings, G_FLAG_USERPREF_NO_SAVE_ON_EXIT);

    if (use_factory_settings) {
      U.runtime.is_dirty = true;
    }
  }

  if (G.fileflags & G_FILE_NO_UI) {
    ED_outliner_select_sync_from_all_tag(C);
  }

  return OPERATOR_FINISHED;
}

static void wm_homefile_read_after_dialog_callback(bContext *C, void *user_data)
{
  WM_operator_name_call_with_properties(
      C, "WM_OT_read_homefile", WM_OP_EXEC_DEFAULT, (IDProperty *)user_data, nullptr);
}

static int wm_homefile_read_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  if (wm_operator_close_file_dialog_if_needed(C, op, wm_homefile_read_after_dialog_callback)) {
    return OPERATOR_INTERFACE;
  }
  return wm_homefile_read_exec(C, op);
}

static void read_homefile_props(wmOperatorType *ot)
{
  PropertyRNA *prop;

  prop = RNA_def_string(ot->srna, "app_template", "Template", sizeof(U.app_template), "", "");
  RNA_def_property_flag(prop, PropertyFlag(PROP_HIDDEN | PROP_SKIP_SAVE));

  prop = RNA_def_boolean(ot->srna, "use_empty", false, "Empty", "");
  RNA_def_property_flag(prop, PropertyFlag(PROP_HIDDEN | PROP_SKIP_SAVE));
}

void WM_OT_read_homefile(wmOperatorType *ot)
{
  PropertyRNA *prop;
  ot->name = "Reload Start-Up File";
  ot->idname = "WM_OT_read_homefile";
  ot->description = "Open the default file (doesn't save the current file)";

  ot->invoke = wm_homefile_read_invoke;
  ot->exec = wm_homefile_read_exec;

  prop = RNA_def_string_file_path(ot->srna,
                                  "filepath",
                                  nullptr,
                                  FILE_MAX,
                                  "File Path",
                                  "Path to an alternative start-up file");
  RNA_def_property_flag(prop, PROP_HIDDEN);

  /* So scripts can use an alternative start-up file without the UI */
  prop = RNA_def_boolean(
      ot->srna, "load_ui", true, "Load UI", "Load user interface setup from the .blend file");
  RNA_def_property_flag(prop, PropertyFlag(PROP_HIDDEN | PROP_SKIP_SAVE));

  /* So the splash can be kept open after loading a file (for templates). */
  prop = RNA_def_boolean(ot->srna, "use_splash", false, "Splash", "");
  RNA_def_property_flag(prop, PropertyFlag(PROP_HIDDEN | PROP_SKIP_SAVE));

  /* So scripts can load factory-startup without resetting preferences
   * (which has other implications such as reloading all add-ons).
   * Match naming for `--factory-startup` command line argument. */
  prop = RNA_def_boolean(ot->srna, "use_factory_startup", false, "Factory Startup", "");
  RNA_def_property_flag(prop, PropertyFlag(PROP_HIDDEN | PROP_SKIP_SAVE));
  read_factory_reset_props(ot);

  read_homefile_props(ot);

  /* omit poll to run in background mode */
}

void WM_OT_read_factory_settings(wmOperatorType *ot)
{
  ot->name = "Load Factory Settings";
  ot->idname = "WM_OT_read_factory_settings";
  ot->description =
      "Load factory default startup file and preferences. "
      "To make changes permanent, use \"Save Startup File\" and \"Save Preferences\"";

  ot->invoke = WM_operator_confirm;
  ot->exec = wm_homefile_read_exec;
  /* Omit poll to run in background mode. */

  read_factory_reset_props(ot);

  read_homefile_props(ot);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Open Main .blend File Utilities
 * \{ */

/**
 * Wrap #WM_file_read, shared by file reading operators.
 */
static bool wm_file_read_opwrap(bContext *C, const char *filepath, ReportList *reports)
{
  /* XXX wm in context is not set correctly after WM_file_read -> crash */
  /* do it before for now, but is this correct with multiple windows? */
  WM_event_add_notifier(C, NC_WINDOW, nullptr);

  /* Set by the "use_scripts" property on file load. */
  if ((G.f & G_FLAG_SCRIPT_AUTOEXEC) == 0) {
    WM_file_autoexec_init(filepath);
  }

  const bool success = WM_file_read(C, filepath, reports);

  return success;
}

/* Generic operator state utilities */

static void create_operator_state(wmOperatorType *ot, int first_state)
{
  PropertyRNA *prop = RNA_def_int(
      ot->srna, "state", first_state, INT32_MIN, INT32_MAX, "State", "", INT32_MIN, INT32_MAX);
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  RNA_def_property_flag(prop, PROP_HIDDEN);
}

static int get_operator_state(wmOperator *op)
{
  return RNA_int_get(op->ptr, "state");
}

static void set_next_operator_state(wmOperator *op, int state)
{
  RNA_int_set(op->ptr, "state", state);
}

typedef struct OperatorDispatchTarget {
  int state;
  int (*run)(bContext *C, wmOperator *op);
} OperatorDispatchTarget;

static int operator_state_dispatch(bContext *C, wmOperator *op, OperatorDispatchTarget *targets)
{
  int state = get_operator_state(op);
  for (int i = 0; targets[i].run; i++) {
    OperatorDispatchTarget target = targets[i];
    if (target.state == state) {
      return target.run(C, op);
    }
  }
  BLI_assert_unreachable();
  return OPERATOR_CANCELLED;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Open Main .blend File Operator
 * \{ */

enum {
  OPEN_MAINFILE_STATE_DISCARD_CHANGES,
  OPEN_MAINFILE_STATE_SELECT_FILE_PATH,
  OPEN_MAINFILE_STATE_OPEN,
};

static int wm_open_mainfile_dispatch(bContext *C, wmOperator *op);

static void wm_open_mainfile_after_dialog_callback(bContext *C, void *user_data)
{
  WM_operator_name_call_with_properties(
      C, "WM_OT_open_mainfile", WM_OP_INVOKE_DEFAULT, (IDProperty *)user_data, nullptr);
}

static int wm_open_mainfile__discard_changes(bContext *C, wmOperator *op)
{
  if (RNA_boolean_get(op->ptr, "display_file_selector")) {
    set_next_operator_state(op, OPEN_MAINFILE_STATE_SELECT_FILE_PATH);
  }
  else {
    set_next_operator_state(op, OPEN_MAINFILE_STATE_OPEN);
  }

  if (wm_operator_close_file_dialog_if_needed(C, op, wm_open_mainfile_after_dialog_callback)) {
    return OPERATOR_INTERFACE;
  }
  return wm_open_mainfile_dispatch(C, op);
}

static int wm_open_mainfile__select_file_path(bContext *C, wmOperator *op)
{
  set_next_operator_state(op, OPEN_MAINFILE_STATE_OPEN);

  Main *bmain = CTX_data_main(C);
  const char *blendfile_path = BKE_main_blendfile_path(bmain);

  if (CTX_wm_window(C) == nullptr) {
    /* in rare cases this could happen, when trying to invoke in background
     * mode on load for example. Don't use poll for this because exec()
     * can still run without a window */
    BKE_report(op->reports, RPT_ERROR, "Context window not set");
    return OPERATOR_CANCELLED;
  }

  /* if possible, get the name of the most recently used .blend file */
  if (G.recent_files.first) {
    RecentFile *recent = static_cast<RecentFile *>(G.recent_files.first);
    blendfile_path = recent->filepath;
  }

  RNA_string_set(op->ptr, "filepath", blendfile_path);
  wm_open_init_load_ui(op, true);
  wm_open_init_use_scripts(op, true);
  op->customdata = nullptr;

  WM_event_add_fileselect(C, op);

  return OPERATOR_RUNNING_MODAL;
}

static int wm_open_mainfile__open(bContext *C, wmOperator *op)
{
  char filepath[FILE_MAX];
  bool success;

  RNA_string_get(op->ptr, "filepath", filepath);

  /* re-use last loaded setting so we can reload a file without changing */
  wm_open_init_load_ui(op, false);
  wm_open_init_use_scripts(op, false);

  SET_FLAG_FROM_TEST(G.fileflags, !RNA_boolean_get(op->ptr, "load_ui"), G_FILE_NO_UI);
  SET_FLAG_FROM_TEST(G.f, RNA_boolean_get(op->ptr, "use_scripts"), G_FLAG_SCRIPT_AUTOEXEC);
  success = wm_file_read_opwrap(C, filepath, op->reports);

  /* for file open also popup for warnings, not only errors */
  BKE_report_print_level_set(op->reports, RPT_WARNING);

  if (success) {
    if (G.fileflags & G_FILE_NO_UI) {
      ED_outliner_select_sync_from_all_tag(C);
    }
    ED_view3d_local_collections_reset(C, (G.fileflags & G_FILE_NO_UI) != 0);
    return OPERATOR_FINISHED;
  }
  return OPERATOR_CANCELLED;
}

static OperatorDispatchTarget wm_open_mainfile_dispatch_targets[] = {
    {OPEN_MAINFILE_STATE_DISCARD_CHANGES, wm_open_mainfile__discard_changes},
    {OPEN_MAINFILE_STATE_SELECT_FILE_PATH, wm_open_mainfile__select_file_path},
    {OPEN_MAINFILE_STATE_OPEN, wm_open_mainfile__open},
    {0, nullptr},
};

static int wm_open_mainfile_dispatch(bContext *C, wmOperator *op)
{
  return operator_state_dispatch(C, op, wm_open_mainfile_dispatch_targets);
}

static int wm_open_mainfile_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  return wm_open_mainfile_dispatch(C, op);
}

static int wm_open_mainfile_exec(bContext *C, wmOperator *op)
{
  return wm_open_mainfile__open(C, op);
}

static char *wm_open_mainfile_description(struct bContext * /*C*/,
                                          struct wmOperatorType * /*op*/,
                                          struct PointerRNA *params)
{
  if (!RNA_struct_property_is_set(params, "filepath")) {
    return nullptr;
  }

  char filepath[FILE_MAX];
  RNA_string_get(params, "filepath", filepath);

  BLI_stat_t stats;
  if (BLI_stat(filepath, &stats) == -1) {
    return BLI_sprintfN("%s\n\n%s", filepath, TIP_("File Not Found"));
  }

  /* Date. */
  char date_st[FILELIST_DIRENTRY_DATE_LEN];
  char time_st[FILELIST_DIRENTRY_TIME_LEN];
  bool is_today, is_yesterday;
  BLI_filelist_entry_datetime_to_string(
      nullptr, int64_t(stats.st_mtime), false, time_st, date_st, &is_today, &is_yesterday);
  if (is_today || is_yesterday) {
    BLI_strncpy(date_st, is_today ? TIP_("Today") : TIP_("Yesterday"), sizeof(date_st));
  }

  /* Size. */
  char size_str[FILELIST_DIRENTRY_SIZE_LEN];
  BLI_filelist_entry_size_to_string(nullptr, uint64_t(stats.st_size), false, size_str);

  return BLI_sprintfN("%s\n\n%s: %s %s\n%s: %s",
                      filepath,
                      TIP_("Modified"),
                      date_st,
                      time_st,
                      TIP_("Size"),
                      size_str);
}

/* currently fits in a pointer */
struct FileRuntime {
  bool is_untrusted;
};
BLI_STATIC_ASSERT(sizeof(struct FileRuntime) <= sizeof(void *),
                  "Struct must not exceed pointer size");

static bool wm_open_mainfile_check(bContext * /*C*/, wmOperator *op)
{
  struct FileRuntime *file_info = (struct FileRuntime *)&op->customdata;
  PropertyRNA *prop = RNA_struct_find_property(op->ptr, "use_scripts");
  bool is_untrusted = false;
  char filepath[FILE_MAX];
  char *lslash;

  RNA_string_get(op->ptr, "filepath", filepath);

  /* get the dir */
  lslash = (char *)BLI_path_slash_rfind(filepath);
  if (lslash) {
    *(lslash + 1) = '\0';
  }

  if ((U.flag & USER_SCRIPT_AUTOEXEC_DISABLE) == 0) {
    if (BKE_autoexec_match(filepath) == true) {
      RNA_property_boolean_set(op->ptr, prop, false);
      is_untrusted = true;
    }
  }

  if (file_info) {
    file_info->is_untrusted = is_untrusted;
  }

  return is_untrusted;
}

static void wm_open_mainfile_ui(bContext * /*C*/, wmOperator *op)
{
  struct FileRuntime *file_info = (struct FileRuntime *)&op->customdata;
  uiLayout *layout = op->layout;
  const char *autoexec_text;

  uiItemR(layout, op->ptr, "load_ui", 0, nullptr, ICON_NONE);

  uiLayout *col = uiLayoutColumn(layout, false);
  if (file_info->is_untrusted) {
    autoexec_text = IFACE_("Trusted Source [Untrusted Path]");
    uiLayoutSetActive(col, false);
    uiLayoutSetEnabled(col, false);
  }
  else {
    autoexec_text = IFACE_("Trusted Source");
  }

  uiItemR(col, op->ptr, "use_scripts", 0, autoexec_text, ICON_NONE);
}

static void wm_open_mainfile_def_property_use_scripts(wmOperatorType *ot)
{
  RNA_def_boolean(ot->srna,
                  "use_scripts",
                  true,
                  "Trusted Source",
                  "Allow .blend file to execute scripts automatically, default available from "
                  "system preferences");
}

void WM_OT_open_mainfile(wmOperatorType *ot)
{
  ot->name = "Open";
  ot->idname = "WM_OT_open_mainfile";
  ot->description = "Open a Blender file";
  ot->get_description = wm_open_mainfile_description;

  ot->invoke = wm_open_mainfile_invoke;
  ot->exec = wm_open_mainfile_exec;
  ot->check = wm_open_mainfile_check;
  ot->ui = wm_open_mainfile_ui;
  /* omit window poll so this can work in background mode */

  WM_operator_properties_filesel(ot,
                                 FILE_TYPE_FOLDER | FILE_TYPE_BLENDER,
                                 FILE_BLENDER,
                                 FILE_OPENFILE,
                                 WM_FILESEL_FILEPATH,
                                 FILE_DEFAULTDISPLAY,
                                 FILE_SORT_DEFAULT);

  RNA_def_boolean(
      ot->srna, "load_ui", true, "Load UI", "Load user interface setup in the .blend file");

  wm_open_mainfile_def_property_use_scripts(ot);

  PropertyRNA *prop = RNA_def_boolean(
      ot->srna, "display_file_selector", true, "Display File Selector", "");
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);

  create_operator_state(ot, OPEN_MAINFILE_STATE_DISCARD_CHANGES);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Reload (revert) Main .blend File Operator
 * \{ */

static int wm_revert_mainfile_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  bool success;
  char filepath[FILE_MAX];

  wm_open_init_use_scripts(op, false);

  SET_FLAG_FROM_TEST(G.f, RNA_boolean_get(op->ptr, "use_scripts"), G_FLAG_SCRIPT_AUTOEXEC);

  BLI_strncpy(filepath, BKE_main_blendfile_path(bmain), sizeof(filepath));
  success = wm_file_read_opwrap(C, filepath, op->reports);

  if (success) {
    return OPERATOR_FINISHED;
  }
  return OPERATOR_CANCELLED;
}

static bool wm_revert_mainfile_poll(bContext * /*C*/)
{
  const char *blendfile_path = BKE_main_blendfile_path_from_global();
  return (blendfile_path[0] != '\0');
}

void WM_OT_revert_mainfile(wmOperatorType *ot)
{
  ot->name = "Revert";
  ot->idname = "WM_OT_revert_mainfile";
  ot->description = "Reload the saved file";

  ot->invoke = WM_operator_confirm;
  ot->exec = wm_revert_mainfile_exec;
  ot->poll = wm_revert_mainfile_poll;

  wm_open_mainfile_def_property_use_scripts(ot);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Recover Last Session Operator
 * \{ */

bool WM_file_recover_last_session(bContext *C, ReportList *reports)
{
  char filepath[FILE_MAX];
  BLI_path_join(filepath, sizeof(filepath), BKE_tempdir_base(), BLENDER_QUIT_FILE);
  G.fileflags |= G_FILE_RECOVER_READ;
  const bool success = wm_file_read_opwrap(C, filepath, reports);
  G.fileflags &= ~G_FILE_RECOVER_READ;
  return success;
}

static int wm_recover_last_session_exec(bContext *C, wmOperator *op)
{
  wm_open_init_use_scripts(op, true);
  SET_FLAG_FROM_TEST(G.f, RNA_boolean_get(op->ptr, "use_scripts"), G_FLAG_SCRIPT_AUTOEXEC);
  if (WM_file_recover_last_session(C, op->reports)) {
    if (!G.background) {
      wmOperatorType *ot = op->type;
      PointerRNA *props_ptr = MEM_cnew<PointerRNA>(__func__);
      WM_operator_properties_create_ptr(props_ptr, ot);
      RNA_boolean_set(props_ptr, "use_scripts", true);
      wm_test_autorun_revert_action_set(ot, props_ptr);
    }
    return OPERATOR_FINISHED;
  }
  return OPERATOR_CANCELLED;
}

static void wm_recover_last_session_after_dialog_callback(bContext *C, void *user_data)
{
  WM_operator_name_call_with_properties(
      C, "WM_OT_recover_last_session", WM_OP_EXEC_DEFAULT, (IDProperty *)user_data, nullptr);
}

static int wm_recover_last_session_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  /* Keep the current setting instead of using the preferences since a file selector
   * doesn't give us the option to change the setting. */
  wm_open_init_use_scripts(op, false);

  if (wm_operator_close_file_dialog_if_needed(
          C, op, wm_recover_last_session_after_dialog_callback)) {
    return OPERATOR_INTERFACE;
  }
  return wm_recover_last_session_exec(C, op);
}

void WM_OT_recover_last_session(wmOperatorType *ot)
{
  ot->name = "Recover Last Session";
  ot->idname = "WM_OT_recover_last_session";
  ot->description = "Open the last closed file (\"" BLENDER_QUIT_FILE "\")";

  ot->invoke = wm_recover_last_session_invoke;
  ot->exec = wm_recover_last_session_exec;

  wm_open_mainfile_def_property_use_scripts(ot);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Auto-Save Main .blend File Operator
 * \{ */

static int wm_recover_auto_save_exec(bContext *C, wmOperator *op)
{
  char filepath[FILE_MAX];
  bool success;

  RNA_string_get(op->ptr, "filepath", filepath);

  wm_open_init_use_scripts(op, true);
  SET_FLAG_FROM_TEST(G.f, RNA_boolean_get(op->ptr, "use_scripts"), G_FLAG_SCRIPT_AUTOEXEC);

  G.fileflags |= G_FILE_RECOVER_READ;

  success = wm_file_read_opwrap(C, filepath, op->reports);

  G.fileflags &= ~G_FILE_RECOVER_READ;

  if (success) {
    if (!G.background) {
      wmOperatorType *ot = op->type;
      PointerRNA *props_ptr = MEM_cnew<PointerRNA>(__func__);
      WM_operator_properties_create_ptr(props_ptr, ot);
      RNA_boolean_set(props_ptr, "use_scripts", true);
      wm_test_autorun_revert_action_set(ot, props_ptr);
    }
    return OPERATOR_FINISHED;
  }
  return OPERATOR_CANCELLED;
}

static int wm_recover_auto_save_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  char filepath[FILE_MAX];

  wm_autosave_location(filepath);
  RNA_string_set(op->ptr, "filepath", filepath);
  wm_open_init_use_scripts(op, true);
  WM_event_add_fileselect(C, op);

  return OPERATOR_RUNNING_MODAL;
}

void WM_OT_recover_auto_save(wmOperatorType *ot)
{
  ot->name = "Recover Auto Save";
  ot->idname = "WM_OT_recover_auto_save";
  ot->description = "Open an automatically saved file to recover it";

  ot->invoke = wm_recover_auto_save_invoke;
  ot->exec = wm_recover_auto_save_exec;

  WM_operator_properties_filesel(ot,
                                 FILE_TYPE_BLENDER,
                                 FILE_BLENDER,
                                 FILE_OPENFILE,
                                 WM_FILESEL_FILEPATH,
                                 FILE_VERTICALDISPLAY,
                                 FILE_SORT_TIME);

  wm_open_mainfile_def_property_use_scripts(ot);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Save Main .blend File Operator
 *
 * Both #WM_OT_save_as_mainfile & #WM_OT_save_mainfile.
 * \{ */

static void wm_filepath_default(const Main *bmain, char *filepath)
{
  if (bmain->filepath[0] == '\0') {
    char filename_untitled[FILE_MAXFILE];
    SNPRINTF(filename_untitled, "%s.blend", DATA_("untitled"));
    BLI_path_filename_ensure(filepath, FILE_MAX, filename_untitled);
  }
}

static void save_set_compress(wmOperator *op)
{
  PropertyRNA *prop;

  prop = RNA_struct_find_property(op->ptr, "compress");
  if (!RNA_property_is_set(op->ptr, prop)) {
    const char *blendfile_path = BKE_main_blendfile_path_from_global();
    if (blendfile_path[0] != '\0') { /* Keep flag for existing file. */
      RNA_property_boolean_set(op->ptr, prop, (G.fileflags & G_FILE_COMPRESS) != 0);
    }
    else { /* use userdef for new file */
      RNA_property_boolean_set(op->ptr, prop, (U.flag & USER_FILECOMPRESS) != 0);
    }
  }
}

static void save_set_filepath(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  PropertyRNA *prop;
  char filepath[FILE_MAX];

  prop = RNA_struct_find_property(op->ptr, "filepath");
  if (!RNA_property_is_set(op->ptr, prop)) {
    const char *blendfile_path = BKE_main_blendfile_path(bmain);
    /* if not saved before, get the name of the most recently used .blend file */
    if ((blendfile_path[0] == '\0') && G.recent_files.first) {
      RecentFile *recent = static_cast<RecentFile *>(G.recent_files.first);
      STRNCPY(filepath, recent->filepath);
    }
    else {
      STRNCPY(filepath, blendfile_path);
    }

    wm_filepath_default(bmain, filepath);
    RNA_property_string_set(op->ptr, prop, filepath);
  }
}

static int wm_save_as_mainfile_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{

  save_set_compress(op);
  save_set_filepath(C, op);

  WM_event_add_fileselect(C, op);

  return OPERATOR_RUNNING_MODAL;
}

/* Function used for #WM_OT_save_mainfile too. */
static int wm_save_as_mainfile_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  char filepath[FILE_MAX];
  const bool is_save_as = (op->type->invoke == wm_save_as_mainfile_invoke);
  const bool use_save_as_copy = is_save_as && RNA_boolean_get(op->ptr, "copy");

  /* We could expose all options to the users however in most cases remapping
   * existing relative paths is a good default.
   * Users can manually make their paths relative & absolute if they wish. */
  const eBLO_WritePathRemap remap_mode = RNA_boolean_get(op->ptr, "relative_remap") ?
                                             BLO_WRITE_PATH_REMAP_RELATIVE :
                                             BLO_WRITE_PATH_REMAP_NONE;
  save_set_compress(op);

  const bool is_filepath_set = RNA_struct_property_is_set(op->ptr, "filepath");
  if (is_filepath_set) {
    RNA_string_get(op->ptr, "filepath", filepath);
  }
  else {
    STRNCPY(filepath, BKE_main_blendfile_path(bmain));
  }

  if (filepath[0] == '\0') {
    BKE_report(op->reports,
               RPT_ERROR,
               "Unable to save an unsaved file with an empty or unset \"filepath\" property");
    return OPERATOR_CANCELLED;
  }

  /* NOTE(@ideasman42): only check this for file-path properties so saving an already
   * saved file never fails with an error.
   * Even though this should never happen, there may be some corner case where a malformed
   * path is stored in `G.main->filepath`: when the file path is initialized from recovering
   * a blend file - for example, so in this case failing to save isn't ideal. */
  if (is_filepath_set && !BLI_path_is_abs_from_cwd(filepath)) {
    BKE_reportf(op->reports,
                RPT_ERROR,
                "The \"filepath\" property was not an absolute path: \"%s\"",
                filepath);
    return OPERATOR_CANCELLED;
  }

  const int fileflags_orig = G.fileflags;
  int fileflags = G.fileflags;

  /* set compression flag */
  SET_FLAG_FROM_TEST(fileflags, RNA_boolean_get(op->ptr, "compress"), G_FILE_COMPRESS);

  const bool success = wm_file_write(
      C, filepath, fileflags, remap_mode, use_save_as_copy, op->reports);

  if ((op->flag & OP_IS_INVOKE) == 0) {
    /* OP_IS_INVOKE is set when the operator is called from the GUI.
     * If it is not set, the operator is called from a script and
     * shouldn't influence G.fileflags. */
    G.fileflags = fileflags_orig;
  }

  if (success == false) {
    return OPERATOR_CANCELLED;
  }

  WM_event_add_notifier(C, NC_WM | ND_FILESAVE, nullptr);

  if (!is_save_as && RNA_boolean_get(op->ptr, "exit")) {
    wm_exit_schedule_delayed(C);
  }

  return OPERATOR_FINISHED;
}

static bool wm_save_mainfile_check(bContext * /*C*/, wmOperator *op)
{
  char filepath[FILE_MAX];
  RNA_string_get(op->ptr, "filepath", filepath);
  if (!BKE_blendfile_extension_check(filepath)) {
    /* some users would prefer BLI_path_extension_replace(),
     * we keep getting nitpicking bug reports about this - campbell */
    BLI_path_extension_ensure(filepath, FILE_MAX, ".blend");
    RNA_string_set(op->ptr, "filepath", filepath);
    return true;
  }
  return false;
}

static const char *wm_save_as_mainfile_get_name(wmOperatorType *ot, PointerRNA *ptr)
{
  if (RNA_boolean_get(ptr, "copy")) {
    return CTX_IFACE_(ot->translation_context, "Save Copy");
  }
  return nullptr;
}

static char *wm_save_as_mainfile_get_description(bContext * /*C*/,
                                                 wmOperatorType * /*ot*/,
                                                 PointerRNA *ptr)
{
  if (RNA_boolean_get(ptr, "copy")) {
    return BLI_strdup(TIP_(
        "Save the current file in the desired location but do not make the saved file active"));
  }
  return nullptr;
}

void WM_OT_save_as_mainfile(wmOperatorType *ot)
{
  PropertyRNA *prop;

  ot->name = "Save As";
  ot->idname = "WM_OT_save_as_mainfile";
  ot->description = "Save the current file in the desired location";

  ot->invoke = wm_save_as_mainfile_invoke;
  ot->exec = wm_save_as_mainfile_exec;
  ot->get_name = wm_save_as_mainfile_get_name;
  ot->get_description = wm_save_as_mainfile_get_description;
  ot->check = wm_save_mainfile_check;
  /* omit window poll so this can work in background mode */

  WM_operator_properties_filesel(ot,
                                 FILE_TYPE_FOLDER | FILE_TYPE_BLENDER,
                                 FILE_BLENDER,
                                 FILE_SAVE,
                                 WM_FILESEL_FILEPATH,
                                 FILE_DEFAULTDISPLAY,
                                 FILE_SORT_DEFAULT);
  RNA_def_boolean(ot->srna, "compress", false, "Compress", "Write compressed .blend file");
  RNA_def_boolean(ot->srna,
                  "relative_remap",
                  true,
                  "Remap Relative",
                  "Remap relative paths when saving to a different directory");
  prop = RNA_def_boolean(
      ot->srna,
      "copy",
      false,
      "Save Copy",
      "Save a copy of the actual working state but does not make saved file active");
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
}

static int wm_save_mainfile_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  int ret;

  /* cancel if no active window */
  if (CTX_wm_window(C) == nullptr) {
    return OPERATOR_CANCELLED;
  }

  save_set_compress(op);
  save_set_filepath(C, op);

  /* if we're saving for the first time and prefer relative paths -
   * any existing paths will be absolute,
   * enable the option to remap paths to avoid confusion #37240. */
  const char *blendfile_path = BKE_main_blendfile_path_from_global();
  if ((blendfile_path[0] == '\0') && (U.flag & USER_RELPATHS)) {
    PropertyRNA *prop = RNA_struct_find_property(op->ptr, "relative_remap");
    if (!RNA_property_is_set(op->ptr, prop)) {
      RNA_property_boolean_set(op->ptr, prop, true);
    }
  }

  if (blendfile_path[0] != '\0') {
    ret = wm_save_as_mainfile_exec(C, op);
  }
  else {
    WM_event_add_fileselect(C, op);
    ret = OPERATOR_RUNNING_MODAL;
  }

  return ret;
}

void WM_OT_save_mainfile(wmOperatorType *ot)
{
  ot->name = "Save Blender File";
  ot->idname = "WM_OT_save_mainfile";
  ot->description = "Save the current Blender file";

  ot->invoke = wm_save_mainfile_invoke;
  ot->exec = wm_save_as_mainfile_exec;
  ot->check = wm_save_mainfile_check;
  /* Omit window poll so this can work in background mode. */

  PropertyRNA *prop;
  WM_operator_properties_filesel(ot,
                                 FILE_TYPE_FOLDER | FILE_TYPE_BLENDER,
                                 FILE_BLENDER,
                                 FILE_SAVE,
                                 WM_FILESEL_FILEPATH,
                                 FILE_DEFAULTDISPLAY,
                                 FILE_SORT_DEFAULT);
  RNA_def_boolean(ot->srna, "compress", false, "Compress", "Write compressed .blend file");
  RNA_def_boolean(ot->srna,
                  "relative_remap",
                  false,
                  "Remap Relative",
                  "Remap relative paths when saving to a different directory");

  prop = RNA_def_boolean(ot->srna, "exit", false, "Exit", "Exit Blender after saving");
  RNA_def_property_flag(prop, PropertyFlag(PROP_HIDDEN | PROP_SKIP_SAVE));
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Auto Script Execution Warning Dialog
 * \{ */

static void wm_block_autorun_warning_ignore(bContext *C, void *arg_block, void * /*arg*/)
{
  wmWindow *win = CTX_wm_window(C);
  UI_popup_block_close(C, win, static_cast<uiBlock *>(arg_block));

  /* Free the data as it's no longer needed. */
  wm_test_autorun_revert_action_set(nullptr, nullptr);
}

static void wm_block_autorun_warning_reload_with_scripts(bContext *C,
                                                         void *arg_block,
                                                         void * /*arg*/)
{
  wmWindow *win = CTX_wm_window(C);

  UI_popup_block_close(C, win, static_cast<uiBlock *>(arg_block));

  /* Save user preferences for permanent execution. */
  if ((U.flag & USER_SCRIPT_AUTOEXEC_DISABLE) == 0) {
    WM_operator_name_call(C, "WM_OT_save_userpref", WM_OP_EXEC_DEFAULT, nullptr, nullptr);
  }

  /* Load file again with scripts enabled.
   * The reload is necessary to allow scripts to run when the files loads. */
  wm_test_autorun_revert_action_exec(C);
}

static void wm_block_autorun_warning_enable_scripts(bContext *C, void *arg_block, void * /*arg*/)
{
  wmWindow *win = CTX_wm_window(C);
  Main *bmain = CTX_data_main(C);

  UI_popup_block_close(C, win, static_cast<uiBlock *>(arg_block));

  /* Save user preferences for permanent execution. */
  if ((U.flag & USER_SCRIPT_AUTOEXEC_DISABLE) == 0) {
    WM_operator_name_call(C, "WM_OT_save_userpref", WM_OP_EXEC_DEFAULT, nullptr, nullptr);
  }

  /* Force a full refresh, but without reloading the file. */
  LISTBASE_FOREACH (Scene *, scene, &bmain->scenes) {
    BKE_scene_free_depsgraph_hash(scene);
  }
}

/* Build the autorun warning dialog UI */
static uiBlock *block_create_autorun_warning(struct bContext *C,
                                             struct ARegion *region,
                                             void * /*arg1*/)
{
  const char *blendfile_path = BKE_main_blendfile_path_from_global();
  wmWindowManager *wm = CTX_wm_manager(C);

  uiBlock *block = UI_block_begin(C, region, "autorun_warning_popup", UI_EMBOSS);
  UI_block_flag_enable(
      block, UI_BLOCK_KEEP_OPEN | UI_BLOCK_LOOP | UI_BLOCK_NO_WIN_CLIP | UI_BLOCK_NUMSELECT);
  UI_block_theme_style_set(block, UI_BLOCK_THEME_STYLE_POPUP);
  UI_block_emboss_set(block, UI_EMBOSS);

  uiLayout *layout = uiItemsAlertBox(block, 44, ALERT_ICON_ERROR);

  /* Title and explanation text. */
  uiLayout *col = uiLayoutColumn(layout, true);
  uiItemL_ex(col,
             TIP_("For security reasons, automatic execution of Python scripts "
                  "in this file was disabled:"),
             ICON_NONE,
             true,
             false);
  uiItemL_ex(col, G.autoexec_fail, ICON_NONE, false, true);
  uiItemL(col, TIP_("This may lead to unexpected behavior"), ICON_NONE);

  uiItemS(layout);

  PointerRNA pref_ptr;
  RNA_pointer_create(nullptr, &RNA_PreferencesFilePaths, &U, &pref_ptr);
  uiItemR(layout,
          &pref_ptr,
          "use_scripts_auto_execute",
          0,
          TIP_("Permanently allow execution of scripts"),
          ICON_NONE);

  uiItemS_ex(layout, 3.0f);

  /* Buttons */
  uiBut *but;
  uiLayout *split = uiLayoutSplit(layout, 0.0f, true);
  uiLayoutSetScaleY(split, 1.2f);

  /* empty space */
  col = uiLayoutColumn(split, false);
  uiItemS(col);

  col = uiLayoutColumn(split, false);

  /* Allow reload if we have a saved file.
   * Otherwise just enable scripts and reset the depsgraphs. */
  if ((blendfile_path[0] != '\0') && wm->file_saved) {
    but = uiDefIconTextBut(block,
                           UI_BTYPE_BUT,
                           0,
                           ICON_NONE,
                           IFACE_("Allow Execution"),
                           0,
                           0,
                           50,
                           UI_UNIT_Y,
                           nullptr,
                           0,
                           0,
                           0,
                           0,
                           TIP_("Reload file with execution of Python scripts enabled"));
    UI_but_func_set(but, wm_block_autorun_warning_reload_with_scripts, block, nullptr);
  }
  else {
    but = uiDefIconTextBut(block,
                           UI_BTYPE_BUT,
                           0,
                           ICON_NONE,
                           IFACE_("Allow Execution"),
                           0,
                           0,
                           50,
                           UI_UNIT_Y,
                           nullptr,
                           0,
                           0,
                           0,
                           0,
                           TIP_("Enable scripts"));
    UI_but_func_set(but, wm_block_autorun_warning_enable_scripts, block, nullptr);
  }
  UI_but_drawflag_disable(but, UI_BUT_TEXT_LEFT);

  col = uiLayoutColumn(split, false);
  but = uiDefIconTextBut(block,
                         UI_BTYPE_BUT,
                         0,
                         ICON_NONE,
                         IFACE_("Ignore"),
                         0,
                         0,
                         50,
                         UI_UNIT_Y,
                         nullptr,
                         0,
                         0,
                         0,
                         0,
                         TIP_("Continue using file without Python scripts"));
  UI_but_func_set(but, wm_block_autorun_warning_ignore, block, nullptr);
  UI_but_drawflag_disable(but, UI_BUT_TEXT_LEFT);
  UI_but_flag_enable(but, UI_BUT_ACTIVE_DEFAULT);

  UI_block_bounds_set_centered(block, 14 * UI_SCALE_FAC);

  return block;
}

/**
 * Store the action needed if the user needs to reload the file with Python scripts enabled.
 *
 * When left to nullptr, this is simply revert.
 * When loading files through the recover auto-save or session,
 * we need to revert using other operators.
 */
static struct {
  wmOperatorType *ot;
  PointerRNA *ptr;
} wm_test_autorun_revert_action_data = {nullptr, nullptr};

void wm_test_autorun_revert_action_set(wmOperatorType *ot, PointerRNA *ptr)
{
  BLI_assert(!G.background);
  wm_test_autorun_revert_action_data.ot = nullptr;
  if (wm_test_autorun_revert_action_data.ptr != nullptr) {
    WM_operator_properties_free(wm_test_autorun_revert_action_data.ptr);
    MEM_freeN(wm_test_autorun_revert_action_data.ptr);
    wm_test_autorun_revert_action_data.ptr = nullptr;
  }
  wm_test_autorun_revert_action_data.ot = ot;
  wm_test_autorun_revert_action_data.ptr = ptr;
}

void wm_test_autorun_revert_action_exec(bContext *C)
{
  wmOperatorType *ot = wm_test_autorun_revert_action_data.ot;
  PointerRNA *ptr = wm_test_autorun_revert_action_data.ptr;

  /* Use regular revert. */
  if (ot == nullptr) {
    ot = WM_operatortype_find("WM_OT_revert_mainfile", false);
    ptr = MEM_cnew<PointerRNA>(__func__);
    WM_operator_properties_create_ptr(ptr, ot);
    RNA_boolean_set(ptr, "use_scripts", true);

    /* Set state, so it's freed correctly */
    wm_test_autorun_revert_action_set(ot, ptr);
  }

  WM_operator_name_call_ptr(C, ot, WM_OP_EXEC_DEFAULT, ptr, nullptr);
  wm_test_autorun_revert_action_set(nullptr, nullptr);
}

void wm_test_autorun_warning(bContext *C)
{
  /* Test if any auto-execution of scripts failed. */
  if ((G.f & G_FLAG_SCRIPT_AUTOEXEC_FAIL) == 0) {
    return;
  }

  /* Only show the warning once. */
  if (G.f & G_FLAG_SCRIPT_AUTOEXEC_FAIL_QUIET) {
    return;
  }

  G.f |= G_FLAG_SCRIPT_AUTOEXEC_FAIL_QUIET;

  wmWindowManager *wm = CTX_wm_manager(C);
  wmWindow *win = (wm->winactive) ? wm->winactive : static_cast<wmWindow *>(wm->windows.first);

  if (win) {
    wmWindow *prevwin = CTX_wm_window(C);
    CTX_wm_window_set(C, win);
    UI_popup_block_invoke(C, block_create_autorun_warning, nullptr, nullptr);
    CTX_wm_window_set(C, prevwin);
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Close File Dialog
 * \{ */

static char save_images_when_file_is_closed = true;

static void wm_block_file_close_cancel(bContext *C, void *arg_block, void * /*arg_data*/)
{
  wmWindow *win = CTX_wm_window(C);
  UI_popup_block_close(C, win, static_cast<uiBlock *>(arg_block));
}

static void wm_block_file_close_discard(bContext *C, void *arg_block, void *arg_data)
{
  wmGenericCallback *callback = WM_generic_callback_steal((wmGenericCallback *)arg_data);

  /* Close the popup before executing the callback. Otherwise
   * the popup might be closed by the callback, which will lead
   * to a crash. */
  wmWindow *win = CTX_wm_window(C);
  UI_popup_block_close(C, win, static_cast<uiBlock *>(arg_block));

  callback->exec(C, callback->user_data);
  WM_generic_callback_free(callback);
}

static void wm_block_file_close_save(bContext *C, void *arg_block, void *arg_data)
{
  const Main *bmain = CTX_data_main(C);
  wmGenericCallback *callback = WM_generic_callback_steal((wmGenericCallback *)arg_data);
  bool execute_callback = true;

  wmWindow *win = CTX_wm_window(C);
  UI_popup_block_close(C, win, static_cast<uiBlock *>(arg_block));

  int modified_images_count = ED_image_save_all_modified_info(CTX_data_main(C), nullptr);
  if (modified_images_count > 0 && save_images_when_file_is_closed) {
    if (ED_image_should_save_modified(bmain)) {
      ReportList *reports = CTX_wm_reports(C);
      ED_image_save_all_modified(C, reports);
      WM_report_banner_show();
    }
    else {
      execute_callback = false;
    }
  }

  bool file_has_been_saved_before = BKE_main_blendfile_path(bmain)[0] != '\0';

  if (file_has_been_saved_before) {
    if (WM_operator_name_call(C, "WM_OT_save_mainfile", WM_OP_EXEC_DEFAULT, nullptr, nullptr) &
        OPERATOR_CANCELLED) {
      execute_callback = false;
    }
  }
  else {
    WM_operator_name_call(C, "WM_OT_save_mainfile", WM_OP_INVOKE_DEFAULT, nullptr, nullptr);
    execute_callback = false;
  }

  if (execute_callback) {
    callback->exec(C, callback->user_data);
  }
  WM_generic_callback_free(callback);
}

static void wm_block_file_close_cancel_button(uiBlock *block, wmGenericCallback *post_action)
{
  uiBut *but = uiDefIconTextBut(
      block, UI_BTYPE_BUT, 0, 0, IFACE_("Cancel"), 0, 0, 0, UI_UNIT_Y, 0, 0, 0, 0, 0, "");
  UI_but_func_set(but, wm_block_file_close_cancel, block, post_action);
  UI_but_drawflag_disable(but, UI_BUT_TEXT_LEFT);
}

static void wm_block_file_close_discard_button(uiBlock *block, wmGenericCallback *post_action)
{
  uiBut *but = uiDefIconTextBut(
      block, UI_BTYPE_BUT, 0, 0, IFACE_("Don't Save"), 0, 0, 0, UI_UNIT_Y, 0, 0, 0, 0, 0, "");
  UI_but_func_set(but, wm_block_file_close_discard, block, post_action);
  UI_but_drawflag_disable(but, UI_BUT_TEXT_LEFT);
}

static void wm_block_file_close_save_button(uiBlock *block, wmGenericCallback *post_action)
{
  uiBut *but = uiDefIconTextBut(
      block, UI_BTYPE_BUT, 0, 0, IFACE_("Save"), 0, 0, 0, UI_UNIT_Y, 0, 0, 0, 0, 0, "");
  UI_but_func_set(but, wm_block_file_close_save, block, post_action);
  UI_but_drawflag_disable(but, UI_BUT_TEXT_LEFT);
  UI_but_flag_enable(but, UI_BUT_ACTIVE_DEFAULT);
}

static const char *close_file_dialog_name = "file_close_popup";

static void save_catalogs_when_file_is_closed_set_fn(bContext * /*C*/, void *arg1, void * /*arg2*/)
{
  char *save_catalogs_when_file_is_closed = static_cast<char *>(arg1);
  ED_asset_catalogs_set_save_catalogs_when_file_is_saved(*save_catalogs_when_file_is_closed != 0);
}

static uiBlock *block_create__close_file_dialog(struct bContext *C,
                                                struct ARegion *region,
                                                void *arg1)
{
  wmGenericCallback *post_action = (wmGenericCallback *)arg1;
  Main *bmain = CTX_data_main(C);

  uiBlock *block = UI_block_begin(C, region, close_file_dialog_name, UI_EMBOSS);
  UI_block_flag_enable(
      block, UI_BLOCK_KEEP_OPEN | UI_BLOCK_LOOP | UI_BLOCK_NO_WIN_CLIP | UI_BLOCK_NUMSELECT);
  UI_block_theme_style_set(block, UI_BLOCK_THEME_STYLE_POPUP);

  uiLayout *layout = uiItemsAlertBox(block, 34, ALERT_ICON_QUESTION);

  /* Title. */
  uiItemL_ex(layout, TIP_("Save changes before closing?"), ICON_NONE, true, false);

  /* Filename. */
  const char *blendfile_path = BKE_main_blendfile_path(CTX_data_main(C));
  char filename[FILE_MAX];
  if (blendfile_path[0] != '\0') {
    BLI_split_file_part(blendfile_path, filename, sizeof(filename));
  }
  else {
    SNPRINTF(filename, "%s.blend", DATA_("untitled"));
  }
  uiItemL(layout, filename, ICON_NONE);

  /* Image Saving Warnings. */
  ReportList reports;
  BKE_reports_init(&reports, RPT_STORE);
  uint modified_images_count = ED_image_save_all_modified_info(bmain, &reports);

  LISTBASE_FOREACH (Report *, report, &reports.list) {
    uiLayout *row = uiLayoutColumn(layout, false);
    uiLayoutSetScaleY(row, 0.6f);
    uiItemS(row);

    /* Error messages created in ED_image_save_all_modified_info() can be long,
     * but are made to separate into two parts at first colon between text and paths.
     */
    char *message = BLI_strdupn(report->message, report->len);
    char *path_info = strstr(message, ": ");
    if (path_info) {
      /* Terminate message string at colon. */
      path_info[1] = '\0';
      /* Skip over the ": " */
      path_info += 2;
    }
    uiItemL_ex(row, message, ICON_NONE, false, true);
    if (path_info) {
      uiItemL_ex(row, path_info, ICON_NONE, false, true);
    }
    MEM_freeN(message);
  }

  /* Used to determine if extra separators are needed. */
  bool has_extra_checkboxes = false;

  /* Modified Images Checkbox. */
  if (modified_images_count > 0) {
    char message[64];
    BLI_snprintf(message, sizeof(message), "Save %u modified image(s)", modified_images_count);
    /* Only the first checkbox should get extra separation. */
    if (!has_extra_checkboxes) {
      uiItemS(layout);
    }
    uiDefButBitC(block,
                 UI_BTYPE_CHECKBOX,
                 1,
                 0,
                 message,
                 0,
                 0,
                 0,
                 UI_UNIT_Y,
                 &save_images_when_file_is_closed,
                 0,
                 0,
                 0,
                 0,
                 "");
    has_extra_checkboxes = true;
  }

  if (AS_asset_library_has_any_unsaved_catalogs()) {
    static char save_catalogs_when_file_is_closed;

    save_catalogs_when_file_is_closed = ED_asset_catalogs_get_save_catalogs_when_file_is_saved();

    /* Only the first checkbox should get extra separation. */
    if (!has_extra_checkboxes) {
      uiItemS(layout);
    }
    uiBut *but = uiDefButBitC(block,
                              UI_BTYPE_CHECKBOX,
                              1,
                              0,
                              "Save modified asset catalogs",
                              0,
                              0,
                              0,
                              UI_UNIT_Y,
                              &save_catalogs_when_file_is_closed,
                              0,
                              0,
                              0,
                              0,
                              "");
    UI_but_func_set(but,
                    save_catalogs_when_file_is_closed_set_fn,
                    &save_catalogs_when_file_is_closed,
                    nullptr);
    has_extra_checkboxes = true;
  }

  BKE_reports_clear(&reports);

  uiItemS_ex(layout, has_extra_checkboxes ? 2.0f : 4.0f);

  /* Buttons. */
#ifdef _WIN32
  const bool windows_layout = true;
#else
  const bool windows_layout = false;
#endif

  if (windows_layout) {
    /* Windows standard layout. */

    uiLayout *split = uiLayoutSplit(layout, 0.0f, true);
    uiLayoutSetScaleY(split, 1.2f);

    uiLayoutColumn(split, false);
    wm_block_file_close_save_button(block, post_action);

    uiLayoutColumn(split, false);
    wm_block_file_close_discard_button(block, post_action);

    uiLayoutColumn(split, false);
    wm_block_file_close_cancel_button(block, post_action);
  }
  else {
    /* Non-Windows layout (macOS and Linux). */

    uiLayout *split = uiLayoutSplit(layout, 0.3f, true);
    uiLayoutSetScaleY(split, 1.2f);

    uiLayoutColumn(split, false);
    wm_block_file_close_discard_button(block, post_action);

    uiLayout *split_right = uiLayoutSplit(split, 0.1f, true);

    uiLayoutColumn(split_right, false);
    /* Empty space. */

    uiLayoutColumn(split_right, false);
    wm_block_file_close_cancel_button(block, post_action);

    uiLayoutColumn(split_right, false);
    wm_block_file_close_save_button(block, post_action);
  }

  UI_block_bounds_set_centered(block, 14 * UI_SCALE_FAC);
  return block;
}

static void free_post_file_close_action(void *arg)
{
  wmGenericCallback *action = (wmGenericCallback *)arg;
  WM_generic_callback_free(action);
}

void wm_close_file_dialog(bContext *C, wmGenericCallback *post_action)
{
  if (!UI_popup_block_name_exists(CTX_wm_screen(C), close_file_dialog_name)) {
    UI_popup_block_invoke(
        C, block_create__close_file_dialog, post_action, free_post_file_close_action);
  }
  else {
    WM_generic_callback_free(post_action);
  }
}

static void wm_free_operator_properties_callback(void *user_data)
{
  IDProperty *properties = (IDProperty *)user_data;
  IDP_FreeProperty(properties);
}

bool wm_operator_close_file_dialog_if_needed(bContext *C,
                                             wmOperator *op,
                                             wmGenericCallbackFn post_action_fn)
{
  if (U.uiflag & USER_SAVE_PROMPT &&
      wm_file_or_session_data_has_unsaved_changes(CTX_data_main(C), CTX_wm_manager(C))) {
    wmGenericCallback *callback = MEM_cnew<wmGenericCallback>(__func__);
    callback->exec = post_action_fn;
    callback->user_data = IDP_CopyProperty(op->properties);
    callback->free_user_data = wm_free_operator_properties_callback;
    wm_close_file_dialog(C, callback);
    return true;
  }

  return false;
}

/** \} */
