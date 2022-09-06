# Copyright 2016, 2020, Thomas Moulard, Olivier Stasse, Guilhem Saurel, JRL,
# CNRS/AIST, LAAS-CNRS

cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_ORG stack-of-tasks)
set(PROJECT_NAME sot-tiago)
set(PROJECT_DESCRIPTION "dynamic-graph package for Tiago robot")
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(DOXYGEN_USE_TEMPLATE_CSS TRUE)
set(CUSTOM_HEADER_DIR "${PROJECT_NAME}")
set(CXX_DISABLE_WERROR TRUE)

# JRL-cmakemodule setup
include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/lapack.cmake)
include(cmake/ros.cmake)

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
add_project_dependency(dynamic-graph-python 4.0.6 REQUIRED)
add_project_dependency(sot-dynamic-pinocchio REQUIRED)
add_project_dependency(dynamic_graph_bridge REQUIRED)
add_project_dependency(tiago_data)
add_required_dependency(dynamic_graph_bridge_msgs)

# Main Library
add_library(${PROJECT_NAME} INTERFACE)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

# Add subdirectories.
add_subdirectory(src)

if(tiago_data_FOUND)
  install(FILES tests/test.py tests/appli.py tests/stop-controllers.py
          DESTINATION share/${PROJECT_NAME}/tests)
endif(tiago_data_FOUND)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
