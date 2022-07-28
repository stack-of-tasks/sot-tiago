# Copyright 2016, R. Budhiraja, Olivier Stasse, CNRS

set(NAME diff-drive-controller)
set(SOURCES diff_drive_controller.cpp speed_limiter.cpp odometry.cpp
            holonomic-projection.cc)

add_library(${NAME} SHARED ${SOURCES})
if(SUFFIX_SO_VERSION)
  set_target_properties(${NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif(SUFFIX_SO_VERSION)

target_include_directories(${NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${NAME} sot-core::sot-core)

install(
  TARGETS ${NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION ${DYNAMIC_GRAPH_PLUGINDIR})

# build python submodule
string(REPLACE - _ PYTHON_LIBRARY_NAME ${NAME})
dynamic_graph_python_module(
  "sot/tiago/${PYTHON_LIBRARY_NAME}" ${NAME}
  sot-tiago-${PYTHON_LIBRARY_NAME}-wrap SOURCE_PYTHON_MODULE
  ${CMAKE_CURRENT_SOURCE_DIR}/diff_drive_controller-python-module.cc)

# Install Python files.
set(PYTHON_MODULE dynamic_graph/sot/tiago)
python_install_on_site("${PYTHON_MODULE}" "__init__.py")
python_install_on_site("${PYTHON_MODULE}" "tiago.py")

python_install_on_site("${PYTHON_MODULE}/steel" "__init__.py")
python_install_on_site("${PYTHON_MODULE}/steel" "robot.py")
python_install_on_site("${PYTHON_MODULE}/steel" "prologue.py")

# Add the library to wrap the device of Tiago.
set(DEVICE_NAME sot-tiago-device)
add_library(${DEVICE_NAME} SHARED sot-tiago-device.cpp)

# Link the dynamic library containing the SoT with its dependencies.
target_link_libraries(${DEVICE_NAME} sot-core::sot-core)

install(
  TARGETS ${DEVICE_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

# build python submodule
string(REPLACE - _ PYTHON_LIBRARY_NAME ${DEVICE_NAME})
dynamic_graph_python_module(
  "sot/tiago/${PYTHON_LIBRARY_NAME}" ${DEVICE_NAME}
  sot-tiago-${PYTHON_LIBRARY_NAME}-wrap SOURCE_PYTHON_MODULE
  ${CMAKE_CURRENT_SOURCE_DIR}/device-python.cc)

# Add the library to wrap the controller of Tiago.
set(CONTROLLER_NAME sot-tiago-steel-controller)
add_library(${CONTROLLER_NAME} SHARED sot-tiago-controller.cpp
                                      sot-tiago-steel-controller.cpp)

# Link the dynamic library containing the SoT with its dependencies.
target_link_libraries(
  ${CONTROLLER_NAME} sot-core::sot-core
  dynamic-graph-python::dynamic-graph-python dynamic_graph_bridge::ros_bridge
  dynamic_graph_bridge::ros_interpreter sot-tiago-device)

install(
  TARGETS ${CONTROLLER_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)
