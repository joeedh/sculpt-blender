# ***** BEGIN GPL LICENSE BLOCK *****
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# The Original Code is Copyright (C) 2006, Blender Foundation
# All rights reserved.
# ***** END GPL LICENSE BLOCK *****

set(INC
  .
  ./intern
  ../blenkernel
  ../blenlib
  ../bmesh
  ../blentranslation
  ../makesdna
  ../../../intern/atomic
  ../../../intern/eigen
  ../../../intern/guardedalloc
  ../../../extern/rangetree
)

set(INC_SYS

)

set(SRC
  intern/trimesh.c
  intern/trimesh_log.c
  intern/trimesh_thread.c
  intern/trimesh_conv.c
  intern/trimesh_private.h
  trimesh.h
)

set(LIB
  bf_blenkernel
  bf_blenlib
  extern_rangetree
)

if(WITH_BULLET)
  list(APPEND INC_SYS
    ${BULLET_INCLUDE_DIRS}
  )
  list(APPEND LIB
    extern_bullet

    ${BULLET_LIBRARIES}
  )
  add_definitions(-DWITH_BULLET)
endif()

if(WITH_INTERNATIONAL)
  add_definitions(-DWITH_INTERNATIONAL)
endif()

if(WITH_FREESTYLE)
  add_definitions(-DWITH_FREESTYLE)
endif()

blender_add_lib(bf_trimesh "${SRC}" "${INC}" "${INC_SYS}" "${LIB}")
