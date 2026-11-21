add_library(rocos_dualarm STATIC
    dualarm_config.cpp dualarm_controller.cpp arm_api_context.cpp
    dualarm_http_server.cpp arm_operations.cpp model_bundle.cpp
    calibration_sessions.cpp motion_commands.cpp)
target_include_directories(rocos_dualarm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src
    PRIVATE ${PROJECT_BINARY_DIR})
target_link_libraries(rocos_dualarm PUBLIC rocos::robot PRIVATE tinyxml2)
add_executable(rocosDualArmMain main.cpp)
target_link_libraries(rocosDualArmMain PRIVATE rocos_dualarm)

# Explicit MuJoCo integration client. The Python harness owns the simulator.
add_executable(dualarm_rci_client tests/rci_client.cpp)
target_link_libraries(dualarm_rci_client PRIVATE librocos)

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

# Optional sibling model integration; all tests are offline (no master access).
set(ROCOS_DUALARM_MODEL_DIR "${PROJECT_SOURCE_DIR}/../rocos-mujoco/config/dualarm/urdf"
    CACHE PATH "Dual-arm URDF directory for shoulder coordinate regression")
if(EXISTS "${ROCOS_DUALARM_MODEL_DIR}/dualarm_shoulder.urdf")
    add_executable(dualarm_shoulder_model_test tests/shoulder_model_test.cpp)
    target_include_directories(dualarm_shoulder_model_test PRIVATE
        ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/test)
    target_compile_definitions(dualarm_shoulder_model_test PRIVATE
        ROCOS_DUALARM_MODEL_DIR="${ROCOS_DUALARM_MODEL_DIR}")
    target_link_libraries(dualarm_shoulder_model_test PRIVATE rocos::robot)
    add_test(NAME dualarm_shoulder_model COMMAND dualarm_shoulder_model_test)
    set_tests_properties(dualarm_shoulder_model PROPERTIES TIMEOUT 60)
    add_test(NAME dualarm_shoulder_config COMMAND rocosDualArmMain
        --config ${CMAKE_CURRENT_SOURCE_DIR}/config/app_shoulder.yaml --check_config)
    set_tests_properties(dualarm_shoulder_config PROPERTIES TIMEOUT 30)
endif()
