# Copyright (c) 2011 CNRS
# Author: Florent Lamiraux, Joseph Mirabel
#
# This file is part of hpp-wholebody-step-corba.
# hpp-wholebody-step-corba is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-wholebody-step-corba is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-wholebody-step-corba.  If not, see
# <http://www.gnu.org/licenses/>.

SET(IDL_SOURCES
  problem)

# search for python
FINDPYTHON(2.7 EXACT REQUIRED)

INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/src)
OMNIIDL_INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/idl)
FILE(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/wholebody-step)

FOREACH(IDL ${IDL_SOURCES})
  GENERATE_IDL_CPP (hpp/corbaserver/wholebody-step/${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/wholebody-step)
  GENERATE_IDL_PYTHON (hpp/corbaserver/wholebody-step/${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/wholebody-step)

  INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/wholebody-step/${IDL}.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/corbaserver/wholebody-step)
  INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/wholebody-step/${IDL}_idl.py
    DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/wholebody_step
    )
ENDFOREACH()

ADD_CUSTOM_TARGET(generate_idl_cpp DEPENDS ${ALL_IDL_CPP_STUBS})
ADD_CUSTOM_TARGET(generate_idl_python DEPENDS ${ALL_IDL_PYTHON_STUBS})

IF (NOT CLIENT_ONLY)
  SET(LIBRARY_NAME ${PROJECT_NAME})
  ADD_LIBRARY(${LIBRARY_NAME}
    SHARED
    server.cc
    problem.impl.hh
    problem.impl.cc
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/wholebody-step/problemSK.cc
    )

  ADD_DEPENDENCIES (${LIBRARY_NAME} generate_idl_cpp)
  ADD_DEPENDENCIES (${LIBRARY_NAME} generate_idl_python)

  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-wholebody-step)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-template-corba)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-util)

  INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

  # Standalone corba server
  ADD_EXECUTABLE (hpp-wholebody-step-server hpp-wholebody-step-corba.cc)
  TARGET_LINK_LIBRARIES (hpp-wholebody-step-server ${LIBRARY_NAME})
  PKG_CONFIG_USE_DEPENDENCY(hpp-wholebody-step-server hpp-wholebody-step)
  PKG_CONFIG_USE_DEPENDENCY(hpp-wholebody-step-server hpp-corbaserver)
  PKG_CONFIG_USE_DEPENDENCY(hpp-wholebody-step-server hpp-template-corba)
  PKG_CONFIG_USE_DEPENDENCY(hpp-wholebody-step-server hpp-util)

  INSTALL (TARGETS hpp-wholebody-step-server DESTINATION ${CMAKE_INSTALL_BINDIR})
ENDIF (NOT CLIENT_ONLY)

INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/wholebody_step/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/wholebody_step
  )
INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/wholebody_step/client.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/wholebody_step
  )
