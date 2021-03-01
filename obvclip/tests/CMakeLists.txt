# ideas de aqui. https://cliutils.gitlab.io/modern-cmake/chapters/testing/googletest.html

# mark_as_advanced(
#   BUILD_GMOCK BUILD_GTEST BUILD_SHARED_LIBS
#   gmock_build_tests gtest_build_samples gtest_build_tests
#   gtest_disable_pthreads gtest_force_shared_crt gtest_hide_internal_symbols
#   )

set_target_properties(gtest PROPERTIES FOLDER extern)
set_target_properties(gtest_main PROPERTIES FOLDER extern)
set_target_properties(gmock PROPERTIES FOLDER extern)
set_target_properties(gmock_main PROPERTIES FOLDER extern)

macro(package_add_test TESTNAME)
  # create an exectuable in which the tests will be stored
  add_executable(${TESTNAME} ${ARGN})
  # link the Google test infrastructure, mocking library, and a default main fuction to
  # the test executable.  Remove g_test_main if writing your own main function.
  target_link_libraries(${TESTNAME} gtest gmock gtest_main)
  # gtest_discover_tests replaces gtest_add_tests,
  # see https://cmake.org/cmake/help/v3.10/module/GoogleTest.html for more options to pass to it

    if (NOT MSVC)
  gtest_discover_tests(${TESTNAME}
    # set a working directory so your project root so that you can find test data via paths relative to the project root
    WORKING_DIRECTORY ${PROJECT_DIR}
    PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_DIR}"
    )
    
    endif()
  set_target_properties(${TESTNAME} PROPERTIES FOLDER tests)
endmacro()


macro(package_add_test_with_libraries TESTNAME FILES LIBRARIES TEST_WORKING_DIRECTORY)
  add_executable(${TESTNAME} ${FILES})
  target_link_libraries(${TESTNAME} gtest gmock gtest_main ${LIBRARIES})

  if (NOT MSVC)
   gtest_discover_tests(${TESTNAME}
        WORKING_DIRECTORY ${TEST_WORKING_DIRECTORY}
     PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${TEST_WORKING_DIRECTORY}"
    )
    endif()

  set_target_properties(${TESTNAME} PROPERTIES FOLDER tests CXX_EXTENSIONS OFF )
  target_compile_features(${TESTNAME} PUBLIC cxx_std_11)
endmacro()

#set(TEST_LIBRARIES obvclip qhullcpp qhullstatic_r OgreMain OgreBites OgreRTShaderSystem Eigen3::Eigen)
set(TEST_LIBRARIES vclipogre OgreMain OgreBites OgreRTShaderSystem Eigen3::Eigen  ${QHULL_LIBRARIES})
set(TEST_LIBRARIES_EIGEN vclipogre Eigen3::Eigen )
package_add_test_with_libraries(qhull-test test_qhull.cpp "${TEST_LIBRARIES}" "${PROJECT_DIR}")
package_add_test_with_libraries(eigen-test test_eigen.cpp "${TEST_LIBRARIES_EIGEN}" "${PROJECT_DIR}")
package_add_test_with_libraries(convexpolytope-test test_convex_polytope.cpp "${TEST_LIBRARIES}" "${PROJECT_DIR}")
package_add_test_with_libraries(vclip-test test_vclip.cpp "${TEST_LIBRARIES}" "${PROJECT_DIR}")
package_add_test_with_libraries(cpp-test test_cpp.cpp "${TEST_LIBRARIES}" "${PROJECT_DIR}")

