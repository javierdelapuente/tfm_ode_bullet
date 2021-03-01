cmake_minimum_required(VERSION 3.10)


# Para Ubuntu 20.04 LTS / linux:
# sudo apt install git libxaw7-dev libxrandr-dev libsdl2-dev libzzip-dev


# Para windows 10:
# Instalar https://www.microsoft.com/en-us/download/details.aspx?id=6812
# Además, para compilar en modo Release, es necesario
# que Ogre utilice el fichero que está en extern/SDL2-2.0.10.tar.gz.
# Se debe configurar en extern/ogre/CMake/Dependencies.cmake:186
# para que lo baje de algún sitio web. Si no es modo Release,
# el proyecto va muy lento en Windows.


project(TFM_ODE_BULLET
  VERSION 0.1
  DESCRIPTION "Proyecto Fin de Máster. Posibilidades y mejora\
de los motores físicos ODE y Bullet"
  LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(obvcliphelpers)


if (MSVC)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)   
    SET(USE_MSVC_RUNTIME_LIBRARY_DLL ON CACHE BOOL "So Bullet 3 works on Windows")
    # If we enable BUILD_SHARED_LIBS, bullet3 does not work on Windows
    SET(BUILD_SHARED_LIBS OFF CACHE BOOL "Be careful. BUILD_SHARED_LIBS shared for qhull and bullet3..")
endif()


# Only do these if this is the main project, and not if it is included through add_subdirectory
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)

  set_property(GLOBAL PROPERTY USE_FOLDERS ON)

  # Docs only available if this is the main app
  find_package(Doxygen)
  if(Doxygen_FOUND)
    add_subdirectory(docs)
  else()
    message(STATUS "Doxygen not found, not building docs")
  endif()
endif()


find_package(Git QUIET)


if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
  # Update submodules as needed
  option(GIT_SUBMODULE "Check submodules during build" ON)
  if(GIT_SUBMODULE)
    message(STATUS "Submodule update")
    execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      RESULT_VARIABLE GIT_SUBMOD_RESULT)
    if(NOT GIT_SUBMOD_RESULT EQUAL "0")
      message(FATAL_ERROR "git submodule update --init failed with \
${GIT_SUBMOD_RESULT}, please checkout submodules")
    endif()
  endif()
endif()

add_subdirectory(extern/qhull)
SET(QHULL_LIBRARIES qhullcpp qhullstatic_r)

SET(OGRE_BUILD_COMPONENT_OVERLAY_IMGUI OFF CACHE BOOL "Include dear imgui in Overlays. Slow to compile.")
add_subdirectory(extern/ogre)
SET(OGRE_DIR_CFG "${OGRE_BINARY_DIR}/bin")
if ( MSVC )
  SET(OGRE_DIR_CFG "${OGRE_DIR_CFG}/${CMAKE_BUILD_TYPE}")
endif ( MSVC )
SET(OGRE_RESOURCES_CFG "${OGRE_DIR_CFG}/resources.cfg")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/resources.cfg.in" "")
cat("${OGRE_RESOURCES_CFG}" "${CMAKE_CURRENT_BINARY_DIR}/resources.cfg.in")
file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/resources.cfg.in" "[tfm_ode_bullet]\n")
file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/resources.cfg.in" "FileSystem=./media\n")
configure_file("${CMAKE_CURRENT_BINARY_DIR}/resources.cfg.in" resources.cfg COPYONLY)
SET(OGRE_PLUGINS_CFG "${OGRE_DIR_CFG}/plugins.cfg")
file(COPY ${OGRE_PLUGINS_CFG} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY media DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

if ( MSVC )
  # this is a mess
  file(READ "${CMAKE_CURRENT_BINARY_DIR}/plugins.cfg" filedata)
  string(REGEX REPLACE "PluginFolder=." "PluginFolder=${OGRE_BINARY_DIR}/bin" filedata "${filedata}")
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/plugins.cfg" "${filedata}")
  file(COPY "${CMAKE_CURRENT_BINARY_DIR}/plugins.cfg" DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  file(COPY "${CMAKE_CURRENT_BINARY_DIR}/resources.cfg" DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
  file(COPY media DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
endif()

SET(BUILD_TESTING OFF CACHE BOOL "Eigen build testing")
add_subdirectory(extern/eigen)

SET(BUILD_UNIT_TESTS OFF CACHE BOOL "Bullet3")
SET(BUILD_BULLET3 OFF CACHE BOOL "Bullet3")
SET(BUILD_BULLET2_DEMOS OFF CACHE BOOL "Bullet3")
SET(USE_DOUBLE_PRECISION ON CACHE BOOL "Bullet3. Use double precision")
SET(BUILD_EXTRAS OFF CACHE BOOL "Bullet3. Set when you want to build the extras")
SET(BUILD_CLSOCKET OFF CACHE BOOL "Bullet3. Set when you want to build apps with enet TCP networking support")
add_subdirectory(extern/bullet3)
# for some reason they are not included?
target_include_directories(BulletCollision INTERFACE ${BULLET_PHYSICS_SOURCE_DIR}/src)

SET(ODE_WITH_DEMOS OFF CACHE BOOL "Builds the demo applications and DrawStuff library")
SET(ODE_WITH_LIBCCD ON CACHE BOOL "Use libccd for handling some collision tests absent in ODE.")
SET(ODE_DOUBLE_PRECISION ON CACHE BOOL "ODE. Use double-precision math")
add_subdirectory(extern/ode)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    get_all_targets_recursive(all_targets_eigen extern/eigen)
    get_all_targets_recursive(all_targets_ogre extern/ogre)
    get_all_targets_recursive(all_targets_qhull extern/qhull)
    get_all_targets_recursive(all_targets_ode extern/ode)
    get_all_targets_recursive(all_targets_bullet3 extern/bullet3)
    set(all_targets ${all_targets_eigen} ${all_targets_ogre} ${all_targets_qhull} ${all_targets_bullet3} ${all_targets_ode})
    foreach(current_target ${all_targets} )
        get_target_property(type ${current_target} TYPE)
        if (${type} STREQUAL "EXECUTABLE" OR ${type} STREQUAL "UTILITY"
	          OR ${type} STREQUAL "SHARED_LIBRARY" OR ${type} STREQUAL "STATIC_LIBRARY")
	        set_property(TARGET ${current_target} PROPERTY FOLDER "TargetsExtern")
        endif()
    endforeach()
endif()


SET(TFM_BUILD_TESTS ON CACHE BOOL "Build all tests.")
if(TFM_BUILD_TESTS)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    include(CTest)
    enable_testing()
    include(GoogleTest)
    add_subdirectory(extern/googletest)
endif()

add_library(config INTERFACE)
target_include_directories(config INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
add_library(tfm::config ALIAS config)


add_subdirectory(spikes)
add_subdirectory(obvclip)
add_subdirectory(obugre)

