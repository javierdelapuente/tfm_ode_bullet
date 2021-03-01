SET(spikeobugre_SRCS
	obugre.cpp
    )

add_executable(spikeobugre ${spikeobugre_SRCS})
target_link_libraries(spikeobugre
  Eigen3::Eigen
  OgreMain OgreBites OgreRTShaderSystem
  ODE
  BulletDynamics BulletCollision LinearMath
  )

IF (USE_DOUBLE_PRECISION)
  target_compile_definitions(spikeobugre PUBLIC -DBT_USE_DOUBLE_PRECISION)
ENDIF (USE_DOUBLE_PRECISION)

target_include_directories(spikeobugre PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

SET(spikeclock_SRCS
	clock.cpp
    )

add_executable(spikeclock ${spikeclock_SRCS})
