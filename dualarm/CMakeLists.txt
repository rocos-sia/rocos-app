add_library(rocos_dualarm STATIC
    dualarm_config.cpp dualarm_controller.cpp arm_api_context.cpp
    dualarm_http_server.cpp arm_operations.cpp model_bundle.cpp
    calibration_sessions.cpp)
target_include_directories(rocos_dualarm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src
    PRIVATE ${PROJECT_BINARY_DIR})
target_link_libraries(rocos_dualarm PUBLIC rocos::robot PRIVATE tinyxml2)
add_executable(rocosDualArmMain main.cpp)
target_link_libraries(rocosDualArmMain PRIVATE rocos_dualarm)

add_executable(dualarm_unit_test tests/dualarm_unit_test.cpp)
target_include_directories(dualarm_unit_test PRIVATE ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(dualarm_unit_test PRIVATE rocos_dualarm)
add_test(NAME dualarm_unit_test COMMAND dualarm_unit_test)
set_tests_properties(dualarm_unit_test PROPERTIES TIMEOUT 20)
add_executable(dualarm_calibration_test tests/calibration_test.cpp)
target_include_directories(dualarm_calibration_test PRIVATE ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(dualarm_calibration_test PRIVATE rocos_dualarm)
add_test(NAME dualarm_calibration_test COMMAND dualarm_calibration_test)
set_tests_properties(dualarm_calibration_test PROPERTIES TIMEOUT 20)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(NAME dualarm_config_test COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/config_test.py $<TARGET_FILE:rocosDualArmMain>
    ${CMAKE_CURRENT_SOURCE_DIR}/config/app.yaml)
set_tests_properties(dualarm_config_test PROPERTIES TIMEOUT 40)
