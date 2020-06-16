# Copyright 2016, R. Budhiraja, Olivier Stasse, CNRS

SET(NAME diff-drive-controller)
SET(SOURCES
  diff_drive_controller.cpp
  speed_limiter.cpp
  odometry.cpp
  holonomic-projection.cc)

ADD_LIBRARY(${NAME} SHARED ${SOURCES})
IF(SUFFIX_SO_VERSION)
  SET_TARGET_PROPERTIES(${NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
ENDIF(SUFFIX_SO_VERSION)

TARGET_INCLUDE_DIRECTORIES(${NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
TARGET_LINK_LIBRARIES(${NAME} sot-core::sot-core)

INSTALL(TARGETS ${NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})

# build python submodule
STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${NAME})
DYNAMIC_GRAPH_PYTHON_MODULE("sot/tiago/${PYTHON_LIBRARY_NAME}"
  ${NAME} sot-tiago-${PYTHON_LIBRARY_NAME}-wrap
  SOURCE_PYTHON_MODULE ${CMAKE_CURRENT_SOURCE_DIR}/diff_drive_controller-python-module.cc)

# Install Python files.
SET(PYTHON_MODULE dynamic_graph/sot/tiago)
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "__init__.py")
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "tiago.py")

PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}/steel" "__init__.py")
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}/steel" "robot.py")
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}/steel" "prologue.py")


# Add the library to wrap the device of Tiago.
SET(DEVICE_NAME sot-tiago-device)
ADD_LIBRARY(${DEVICE_NAME} SHARED sot-tiago-device.cpp)

# Link the dynamic library containing the SoT with its dependencies.
TARGET_LINK_LIBRARIES(${DEVICE_NAME} sot-core::sot-core)

IF(UNIX AND NOT APPLE)
  TARGET_LINK_LIBRARIES(${DEVICE_NAME} ${Boost_LIBRARIES})
ENDIF(UNIX AND NOT APPLE)

INSTALL(TARGETS ${DEVICE_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

# build python submodule
STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${DEVICE_NAME})
DYNAMIC_GRAPH_PYTHON_MODULE("sot/tiago/${PYTHON_LIBRARY_NAME}"
  ${DEVICE_NAME} sot-tiago-${PYTHON_LIBRARY_NAME}-wrap
  SOURCE_PYTHON_MODULE ${CMAKE_CURRENT_SOURCE_DIR}/device-python.cc)


# Add the library to wrap the controller of Tiago.
SET(CONTROLLER_NAME sot-tiago-steel-controller)
ADD_LIBRARY(${CONTROLLER_NAME} SHARED
  sot-tiago-controller.cpp
  sot-tiago-steel-controller.cpp
  )

# Link the dynamic library containing the SoT with its dependencies.
TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} sot-core::sot-core dynamic-graph-python::dynamic-graph-python
    dynamic_graph_bridge::ros_bridge dynamic_graph_bridge::ros_interpreter sot-tiago-device)

IF(UNIX AND NOT APPLE)
  TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} ${Boost_LIBRARIES})
ENDIF(UNIX AND NOT APPLE)

INSTALL(TARGETS ${CONTROLLER_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
