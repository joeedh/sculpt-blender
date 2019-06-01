#project(pluginlite)

set(SRC ./intern/stubs.c)

set(INC ../../source/blender/makesdna)

if (WIN32)
    set(SRC 
        ${SRC} 
       ./win32/exports.def
    )
endif()

#set(BKE_PATH "C:/dev/blender/2.8/build_win64/source/blender/blenkernel/bf_blenkernel.dir/RelWithDebInfo")
set(OBJ_SUFFIX ".o")
set(LIB_SUFFIX ".a")
set(LIB_PREFIX "${CMAKE_STATIC_LIBRARY_PREFIX}")

if (MSVC)
    set(OBJ_SUFFIX ".obj")
    set(LIB_SUFFIX ".lib")
endif()

set(BKE_PATH "${PROJECT_BINARY_DIR}/source/blender/blenkernel/bf_blenkernel.dir/${CMAKE_CFG_INTDIR}")
set(BLENLIB_PATH "${PROJECT_BINARY_DIR}/lib/${CMAKE_CFG_INTDIR}")

add_library(pluginlite MODULE ${SRC})
target_include_directories(pluginlite PUBLIC ${INC})

set(LIBS
    bf_intern_guardedalloc
    bf_intern_clog
    bf_intern_eigen
    "${BLENLIB_PATH}/${LIB_PREFIX}bf_bmesh${LIB_SUFFIX}"
    "${BLENLIB_PATH}/${LIB_PREFIX}bf_blenlib${LIB_SUFFIX}"
    "${BKE_PATH}/customdata${OBJ_SUFFIX}"
    "${BKE_PATH}/customdata_file${OBJ_SUFFIX}"
    "${BKE_PATH}/mesh${OBJ_SUFFIX}"
    "${BKE_PATH}/mesh_evaluate${OBJ_SUFFIX}"
    "${BKE_PATH}/mesh_runtime${OBJ_SUFFIX}"
    "${BKE_PATH}/multires${OBJ_SUFFIX}"
    "${BKE_PATH}/idcode${OBJ_SUFFIX}"
)

if(WIN32 AND NOT UNIX)
  set(LIBS
    "${LIBS}"
    "${PTHREADS_LIBRARIES}"
  )
endif()

target_link_libraries(pluginlite "${LIBS}")
 
 #message("test ${BLENLIB_PATH} ${CMAKE_SOURCE_DIR} | ${CMAKE_FILES_DIRECTORY} | ${CMAKE_ROOT} | ${PROJECT_BINARY_DIR}")
 message("test ${BLENLIB_PATH} | ${CMAKE_STATIC_LIBRARY_PREFIX} | ${OBJ_SUFFIX} | ${BKE_PATH} | ${CMAKE_SOURCE_DIR} | ${PROJECT_BINARY_DIR} | ${CMAKE_CFG_INTDIR}")
 
#add_dependencies(pluginlite bf_intern_guardedalloc)

#set(CMAKE_LINK_DEF_FILE_FLAG /DEF:win32/exports.def )
#set(CMAKE_LINK_DEF_FILE_FLAG_DEBUG /DEF:win32/exports.def )
