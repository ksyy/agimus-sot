#
# Copyright (c) 2014 CNRS
# Authors: Florent Lamiraux
#
#
# This file is part of agimus_sot.
# agimus_sot is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# agimus_sot is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# agimus_sot  If not, see
# <http://www.gnu.org/licenses/>.

MACRO(AGIMUS_SOT_PYTHON_MODULE SUBMODULENAME LIBRARYNAME TARGETNAME)
  FINDPYTHON()

  SET(PYTHON_MODULE ${TARGETNAME})

  ADD_LIBRARY(${PYTHON_MODULE}
    MODULE
    ${PROJECT_SOURCE_DIR}/cmake/dynamic_graph/python-module-py.cc)

  FILE(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/src/agimus_sot/${SUBMODULENAME})
  SET_TARGET_PROPERTIES(${PYTHON_MODULE}
    PROPERTIES PREFIX ""
    OUTPUT_NAME agimus_sot/${SUBMODULENAME}/wrap
   )

  TARGET_LINK_LIBRARIES(${PYTHON_MODULE} "-Wl,--no-as-needed")
  TARGET_LINK_LIBRARIES(${PYTHON_MODULE} ${LIBRARYNAME} ${PYTHON_LIBRARY})

  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

  #
  # Installation
  #
  SET(PYTHON_INSTALL_DIR ${PYTHON_SITELIB}/agimus_sot/${SUBMODULENAME})

  INSTALL(TARGETS ${PYTHON_MODULE}
    DESTINATION
    ${PYTHON_INSTALL_DIR})

  SET(ENTITY_CLASS_LIST "")
  FOREACH (ENTITY ${NEW_ENTITY_CLASS})
    SET(ENTITY_CLASS_LIST "${ENTITY_CLASS_LIST}${ENTITY}('')\n")
  ENDFOREACH(ENTITY ${NEW_ENTITY_CLASS})

  CONFIGURE_FILE(
    ${PROJECT_SOURCE_DIR}/cmake/dynamic_graph/submodule/__init__.py.cmake
    ${PROJECT_BINARY_DIR}/src/agimus_sot/${SUBMODULENAME}/__init__.py
    )

  INSTALL(
    FILES ${PROJECT_BINARY_DIR}/src/agimus_sot/${SUBMODULENAME}/__init__.py
    DESTINATION ${PYTHON_INSTALL_DIR}
    )

ENDMACRO()

SET(LIBRARY_NAME ${PROJECT_NAME})

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  holonomic-constraint.cc
  delay.cc
  time.cc
  ros_tf_broadcaster.cc
  )

PKG_CONFIG_USE_DEPENDENCY (${LIBRARY_NAME} dynamic-graph-python)
PKG_CONFIG_USE_DEPENDENCY (${LIBRARY_NAME} dynamic_graph_bridge)
PKG_CONFIG_USE_DEPENDENCY (${LIBRARY_NAME} sot-core)
INSTALL(TARGETS ${LIBRARY_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  )

SET(NEW_ENTITY_CLASS "HolonomicConstraint")

AGIMUS_SOT_PYTHON_MODULE("sot" ${LIBRARY_NAME} wrap)
FILE(WRITE ${CMAKE_CURRENT_BINARY_DIR}/agimus_sot/__init__.py "")
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/agimus_sot/__init__.py
    DESTINATION ${PYTHON_SITELIB}/agimus_sot/)

SET(FILES
  supervisor.py
  tools.py
  solver.py
  events.py
  ros_interface.py
  factory.py
  srdf_parser.py
  __init__.py)

FOREACH(F ${FILES})
    PYTHON_INSTALL_ON_SITE("agimus_sot" ${F})
ENDFOREACH()

SET(FILES
  controllers.py
  switch.py
  gripper.py
  __init__.py)

FOREACH(F ${FILES})
    PYTHON_INSTALL_ON_SITE("agimus_sot/control" ${F})
ENDFOREACH()
