set(OBUGRE_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/system.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/body.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/engine.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/looper.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/renderer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ode_engine.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/engine_logger.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/bullet_engine.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/btConvexConvexVclipAlgorithm.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/objectTextDisplay.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/body_debug.cpp
  # ${CMAKE_CURRENT_SOURCE_DIR}/src/ogreExtras.cpp
  )


add_library(obugre  SHARED STATIC ${OBUGRE_SOURCE_FILES})
target_link_libraries(obugre
  Eigen3::Eigen
  vclip
  OgreMain OgreBites OgreRTShaderSystem
  ODE
  BulletDynamics BulletCollision LinearMath
  )
target_include_directories(obugre PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
target_include_directories(obugre PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
target_include_directories(obugre PRIVATE "${PROJECT_BINARY_DIR}/include")
target_include_directories(obugre PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
IF (USE_DOUBLE_PRECISION)
  target_compile_definitions(obugre PUBLIC -DBT_USE_DOUBLE_PRECISION)
ENDIF (USE_DOUBLE_PRECISION)

add_library(Obvclip::obugre ALIAS obugre)

#message(STATUS "ODE_WITH_LIBCCD: ${ODE_WITH_LIBCCD}")

configure_file (
  "${CMAKE_CURRENT_SOURCE_DIR}/include/configure_obugre.h.in"
  "${PROJECT_BINARY_DIR}/include/configure_obugre.h"
  )

set(TWOBOXES_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/apps/twoboxes.cpp)
add_executable(twoboxes ${TWOBOXES_FILES})
target_link_libraries(twoboxes obugre)


set(SPHERE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/apps/sphere.cpp)
add_executable(sphere ${SPHERE_FILES})
target_link_libraries(sphere obugre)

set(BIGTEST_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/apps/bigtest.cpp)
add_executable(bigtest ${BIGTEST_FILES})
target_link_libraries(bigtest obugre)


set(OBVCLIPTEST_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/apps/obvcliptest.cpp)
add_executable(obvcliptest ${OBVCLIPTEST_FILES})
target_link_libraries(obvcliptest obugre)

set(OBVCLIPTEST2_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/apps/obvcliptest2.cpp)
add_executable(obvcliptest2 ${OBVCLIPTEST2_FILES})
target_link_libraries(obvcliptest2 obugre)


set(PERFORMANCEVCLIP_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/apps/performancevclip.cpp)
add_executable(performancevclip ${PERFORMANCEVCLIP_FILES})
target_link_libraries(performancevclip obugre)


set(CONSTRAINTTTEST_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/apps/constrainttest.cpp)
add_executable(constrainttest ${CONSTRAINTTTEST_FILES})
target_link_libraries(constrainttest obugre)


set(COMPARATIVA1_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/memoria/comparativa1.cpp)
add_executable(comparativa1 ${COMPARATIVA1_FILES})
target_link_libraries(comparativa1 obugre)

set(COMPARATIVA2_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/memoria/comparativa2.cpp)
add_executable(comparativa2 ${COMPARATIVA2_FILES})
target_link_libraries(comparativa2 obugre)

set(COMPARATIVA3_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/memoria/comparativa3.cpp)
add_executable(comparativa3 ${COMPARATIVA3_FILES})
target_link_libraries(comparativa3 obugre)

set(COMPARATIVA4_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/memoria/comparativa4.cpp)
add_executable(comparativa4 ${COMPARATIVA4_FILES})
target_link_libraries(comparativa4 obugre)

set(COMPARATIVA5_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/memoria/comparativa5.cpp)
add_executable(comparativa5 ${COMPARATIVA5_FILES})
target_link_libraries(comparativa5 obugre)


set(CACHE_NO_CACHE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/memoria/performance_cache_nocache.cpp)
add_executable(cache_no_cache ${CACHE_NO_CACHE_FILES})
target_link_libraries(cache_no_cache obugre)


set(PERFORMANCE_VCLIP_ODE_BULLET
  ${CMAKE_CURRENT_SOURCE_DIR}/memoria/performance_vclip_ode_bullet.cpp)
add_executable(vclip_ode_bullet ${PERFORMANCE_VCLIP_ODE_BULLET})
target_link_libraries(vclip_ode_bullet obugre)
