# Copyright 2016, 2020, Thomas Moulard, Olivier Stasse, Guilhem Saurel, JRL, CNRS/AIST, LAAS-CNRS

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

# Project properties
SET(PROJECT_ORG stack-of-tasks)
SET(PROJECT_NAME sot-tiago)
SET(PROJECT_DESCRIPTION "dynamic-graph package for Tiago robot")
SET(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
OPTION(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
SET(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(DOXYGEN_USE_TEMPLATE_CSS TRUE)
SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")
SET(CXX_DISABLE_WERROR TRUE)

# JRL-cmakemodule setup
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/ros.cmake)
INCLUDE(cmake/python.cmake)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
FINDPYTHON()
SEARCH_FOR_BOOST_PYTHON()

ADD_PROJECT_DEPENDENCY(sot-dynamic-pinocchio REQUIRED PKG_CONFIG_REQUIRES sot-dynamic-pinocchio)
ADD_PROJECT_DEPENDENCY(dynamic-graph-python 4.0.0 REQUIRED PKG_CONFIG_REQUIRES dynamic-graph-python)
ADD_PROJECT_DEPENDENCY(dynamic_graph_bridge REQUIRED PKG_CONFIG_REQUIRES dynamic_graph_bridge)
ADD_PROJECT_DEPENDENCY(tiago_data)
ADD_REQUIRED_DEPENDENCY(dynamic_graph_bridge_msgs)

# Main Library
ADD_LIBRARY(${PROJECT_NAME} INTERFACE)
INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

# Add subdirectories.
ADD_SUBDIRECTORY(src)

IF(tiago_data_FOUND)
  INSTALL(FILES
    tests/test.py
    tests/appli.py
    tests/stop-controllers.py
    DESTINATION share/${PROJECT_NAME}/tests)
ENDIF(tiago_data_FOUND)

INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})
