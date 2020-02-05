# Copyright 2016, R. Budhiraja, Olivier Stasse, CNRS

FUNCTION(COMPILE_PLUGIN NAME)
  ADD_LIBRARY(${NAME} SHARED ${SOURCES})
    SET_TARGET_PROPERTIES(${lib} PROPERTIES PREFIX "")
  IF (SUFFIX_SO_VERSION)
    SET_TARGET_PROPERTIES(${lib} PROPERTIES SOVERSION ${PROJECT_VERSION})
  ENDIF()

  PKG_CONFIG_USE_DEPENDENCY(${NAME} dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} sot-core)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} pinocchio)

  INSTALL(TARGETS ${NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR}/plugin)

  # build python submodule
  STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${NAME})
  SET(NEW_ENTITY_CLASS ${ENTITIES})
  DYNAMIC_GRAPH_PYTHON_MODULE("sot/tiago/${PYTHON_LIBRARY_NAME}"
    ${NAME}
    sot-tiago-${PYTHON_LIBRARY_NAME}-wrap
    )
ENDFUNCTION()

SET(ENTITIES DiffDriveController HolonomicProjection)
SET(SOURCES
  diff_drive_controller.cpp
  speed_limiter.cpp
  odometry.cpp
  holonomic-projection.cc)
COMPILE_PLUGIN(diff-drive-controller)
TARGET_INCLUDE_DIRECTORIES(diff-drive-controller
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Install Python files.
SET(PYTHON_MODULE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/tiago)
SET(PYTHON_MODULE_BUILD_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/dynamic_graph/sot/tiago)

SET(PYTHON_MODULE dynamic_graph/sot/tiago)
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "__init__.py" )
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "tiago.py" )
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "robot.py" )
SET(FILES __init__.py robot.py)

# Install dynamic_graph.sot.tiago.steel
SET(PYTHON_MODULE dynamic_graph/sot/tiago/steel)
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "__init__.py" )
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "robot.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL_ON_SITE("${PYTHON_MODULE}" "prologue.py")


# Add the library to wrap the device of Tiago.
MACRO(build_tiago_device)

  SET(DEVICE_NAME sot-tiago-device)
  ADD_LIBRARY(${DEVICE_NAME}
    SHARED
    sot-tiago-device.cpp
  )

  # Link the dynamic library containing the SoT with its dependencies.
  PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "dynamic-graph")
  PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "dynamic-graph-python")
  PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "sot-core")
  PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "pinocchio")
  PKG_CONFIG_USE_DEPENDENCY(${DEVICE_NAME} "dynamic_graph_bridge")

  IF(UNIX AND NOT APPLE)
    TARGET_LINK_LIBRARIES(${DEVICE_NAME} ${Boost_LIBRARIES})
  ENDIF(UNIX AND NOT APPLE)

  INSTALL(TARGETS ${DEVICE_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

  # build python submodule
  STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${DEVICE_NAME})
  SET(NEW_ENTITY_CLASS ${ENTITIES})
  DYNAMIC_GRAPH_PYTHON_MODULE("sot/tiago/${PYTHON_LIBRARY_NAME}"
    ${DEVICE_NAME}
    sot-tiago-${PYTHON_LIBRARY_NAME}-wrap
    )

ENDMACRO()

build_tiago_device()

# Add the library to wrap the controller of Tiago.
MACRO(build_tiago_controller CONTROLLER_NAME)

  ADD_LIBRARY(${CONTROLLER_NAME}
    SHARED
    sot-tiago-controller.cpp
    sot-tiago-steel-controller.cpp
  )

  # Link the dynamic library containing the SoT with its dependencies.
  PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic-graph")
  PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic-graph-python")
  PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "sot-core")

  PKG_CONFIG_USE_DEPENDENCY(${CONTROLLER_NAME} "dynamic_graph_bridge")

  ADD_DEPENDENCIES(${CONTROLLER_NAME} "sot-tiago-device")
  TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} "sot-tiago-device")
  TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} "ros_bridge")
  TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} "ros_interpreter")

  IF(UNIX AND NOT APPLE)
    TARGET_LINK_LIBRARIES(${CONTROLLER_NAME} ${Boost_LIBRARIES})
  ENDIF(UNIX AND NOT APPLE)

  INSTALL(TARGETS ${CONTROLLER_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

ENDMACRO()

build_tiago_controller(sot-tiago-steel-controller TRUE)
