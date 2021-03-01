
set(VCLIP_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/polytope.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/vclip.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/polytope_examples.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/contacts.cpp
  )

add_library(vclip  SHARED STATIC ${VCLIP_SOURCE_FILES})
target_link_libraries(vclip tfm::config ${QHULL_LIBRARIES} Eigen3::Eigen)
target_include_directories(vclip PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
add_library(obvclip::vclip ALIAS vclip)



set(VCLIPOGRE_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/ogrevclip_app.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/MovableText.cpp
  )

add_library(vclipogre  SHARED STATIC ${VCLIPOGRE_SOURCE_FILES})
target_link_libraries(vclipogre vclip OgreMain OgreBites OgreRTShaderSystem)
target_include_directories(vclipogre PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tools>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
add_library(Obvclip::vclipogre ALIAS vclipogre)


set(SOURCE_FILES_SHOWCUBE
  ${CMAKE_CURRENT_SOURCE_DIR}/apps/showcube.cpp
  )
add_executable(showcube ${SOURCE_FILES_SHOWCUBE})
target_link_libraries(showcube vclipogre)


set(SOURCE_FILES_TWOCUBES
  ${CMAKE_CURRENT_SOURCE_DIR}/apps/twocubes.cpp
  )
add_executable(twocubes ${SOURCE_FILES_TWOCUBES})
target_link_libraries(twocubes vclipogre)

set(SOURCE_FILES_OGREBASIC
  ${CMAKE_CURRENT_SOURCE_DIR}/apps/ogrebasic.cpp
  )
add_executable(ogrebasic ${SOURCE_FILES_OGREBASIC})
target_link_libraries(ogrebasic vclipogre)


set(SOURCE_FILES_TWOCOMPLEX
  ${CMAKE_CURRENT_SOURCE_DIR}/apps/twocomplex.cpp
  )
add_executable(twocomplex ${SOURCE_FILES_TWOCOMPLEX})
target_link_libraries(twocomplex vclipogre)


set(SOURCE_FILES_TESTVCLIP
  ${CMAKE_CURRENT_SOURCE_DIR}/apps/testvclip.cpp
  )
add_executable(testvclip ${SOURCE_FILES_TESTVCLIP})
target_link_libraries(testvclip vclipogre)

set(SOURCE_FILES_VCLIPSTANDALONE
  ${CMAKE_CURRENT_SOURCE_DIR}/apps/vclipstandalone.cpp
  )
add_executable(vclipstandalone ${SOURCE_FILES_VCLIPSTANDALONE})
target_link_libraries(vclipstandalone vclipogre)



set(SOURCE_FILES_ANALISIS1
  ${CMAKE_CURRENT_SOURCE_DIR}/memoria/analisis1.cpp
  )
add_executable(analisis1 ${SOURCE_FILES_ANALISIS1})
target_link_libraries(analisis1 vclipogre)

set(SOURCE_FILES_ANALISIS2
  ${CMAKE_CURRENT_SOURCE_DIR}/memoria/analisis2.cpp
  )
add_executable(analisis2 ${SOURCE_FILES_ANALISIS2})
target_link_libraries(analisis2 vclipogre)


set(SOURCE_FILES_ANALISIS3
  ${CMAKE_CURRENT_SOURCE_DIR}/memoria/analisis3.cpp
  )
add_executable(analisis3 ${SOURCE_FILES_ANALISIS3})
target_link_libraries(analisis3 vclipogre)

set(SOURCE_FILES_CONTACTOS
  ${CMAKE_CURRENT_SOURCE_DIR}/memoria/contactos.cpp
  )
add_executable(contactos ${SOURCE_FILES_CONTACTOS})
target_link_libraries(contactos vclipogre)



if(TFM_BUILD_TESTS)
  file(COPY tests/data DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
  file(COPY tests/data DESTINATION "${CMAKE_BINARY_DIR}")
  add_subdirectory(tests)
endif()
