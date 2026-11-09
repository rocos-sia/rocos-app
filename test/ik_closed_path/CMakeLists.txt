cmake_minimum_required(VERSION 3.15)
project(ik_closed_path_comparison LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
get_filename_component(ROCOS_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(ROCOS_BUILD_DIR "${ROCOS_SOURCE_DIR}/build" CACHE PATH "Existing dependency build")
list(PREPEND CMAKE_PREFIX_PATH "${ROCOS_BUILD_DIR}/deps/prefix")
find_package(pinocchio CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time system)
find_package(urdfdom REQUIRED)
find_library(IK_TEST_NLOPT_LIBRARY NAMES nlopt PATHS "${ROCOS_BUILD_DIR}/lib" NO_DEFAULT_PATH)
if(NOT IK_TEST_NLOPT_LIBRARY)
    message(FATAL_ERROR "Build the repository NLopt dependency first, or set ROCOS_BUILD_DIR")
endif()
add_library(ik_test_nlopt SHARED IMPORTED)
set_target_properties(ik_test_nlopt PROPERTIES
    IMPORTED_LOCATION "${IK_TEST_NLOPT_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${ROCOS_SOURCE_DIR}/3rdparty/nlopt/src/api;${ROCOS_BUILD_DIR}/3rdparty/nlopt")

add_library(ik_test_parser STATIC "${ROCOS_SOURCE_DIR}/3rdparty/kdl_parser/src/kdl_parser.cpp")
target_include_directories(ik_test_parser PUBLIC "${ROCOS_SOURCE_DIR}/3rdparty/kdl_parser/include" ${urdfdom_INCLUDE_DIRS})
target_link_libraries(ik_test_parser PUBLIC Eigen3::Eigen orocos-kdl ${urdfdom_LIBRARIES} tinyxml tinyxml2)

add_library(ik_test_pin STATIC
    "${ROCOS_SOURCE_DIR}/3rdparty/pin_ik/src/pin_ik.cpp"
    "${ROCOS_SOURCE_DIR}/3rdparty/pin_ik/src/pinocchio_tl.cpp"
    "${ROCOS_SOURCE_DIR}/3rdparty/pin_ik/src/nlopt_ik.cpp"
    "${ROCOS_SOURCE_DIR}/3rdparty/pin_ik/src/urdf.cpp"
    "${ROCOS_SOURCE_DIR}/3rdparty/pin_ik/src/worker_thread.cpp")
target_include_directories(ik_test_pin PUBLIC "${ROCOS_SOURCE_DIR}/3rdparty/pin_ik/include")
target_link_libraries(ik_test_pin PUBLIC pinocchio::pinocchio Eigen3::Eigen Threads::Threads ik_test_nlopt)

add_library(ik_test_trac STATIC
    "${ROCOS_SOURCE_DIR}/3rdparty/trac_ik/src/trac_ik.cpp"
    "${ROCOS_SOURCE_DIR}/3rdparty/trac_ik/src/kdl_tl.cpp"
    "${ROCOS_SOURCE_DIR}/3rdparty/trac_ik/src/nlopt_ik.cpp")
target_include_directories(ik_test_trac PUBLIC "${ROCOS_SOURCE_DIR}/3rdparty/trac_ik/include")
target_link_libraries(ik_test_trac PUBLIC ik_test_parser ik_test_nlopt Threads::Threads Boost::date_time Boost::system)

# 两个库具有同名 NLOPT_IK 符号，必须在不同可执行程序中运行。
enable_testing()
foreach(backend IN ITEMS pin trac)
    add_executable(${backend}_ik_closed_path_test "${CMAKE_CURRENT_LIST_DIR}/../ik_closed_path_test.cc")
    target_include_directories(${backend}_ik_closed_path_test PRIVATE "${ROCOS_SOURCE_DIR}/3rdparty/json")
    target_compile_definitions(${backend}_ik_closed_path_test PRIVATE
        IK_TEST_${backend}=1
        IK_TEST_URDF="${ROCOS_SOURCE_DIR}/config/robots/talon/robot.urdf"
        IK_TEST_ARM6="${ROCOS_SOURCE_DIR}/3rdparty/pin_ik/tests/benchmark/arm6.urdf")
    target_link_libraries(${backend}_ik_closed_path_test PRIVATE ik_test_${backend} ik_test_parser)
    add_test(NAME ${backend}_ik_closed_path_smoke COMMAND ${backend}_ik_closed_path_test
        "--test-case=Standalone Talon closed path IK comparison")
    set_tests_properties(${backend}_ik_closed_path_smoke PROPERTIES
        ENVIRONMENT "IK_DRIFT_BACKEND=newton;IK_DRIFT_CYCLES=2;IK_DRIFT_OUTPUT=${CMAKE_BINARY_DIR}/${backend}_smoke")
endforeach()

foreach(backend IN ITEMS pin trac)
    add_executable(${backend}_ik_workspace_survey ik_workspace_survey.cc)
    target_include_directories(${backend}_ik_workspace_survey PRIVATE "${ROCOS_SOURCE_DIR}/3rdparty/json")
    target_compile_definitions(${backend}_ik_workspace_survey PRIVATE IK_TEST_${backend}=1
        IK_TEST_URDF="${ROCOS_SOURCE_DIR}/config/robots/talon/robot.urdf"
        IK_TEST_ARM6="${ROCOS_SOURCE_DIR}/3rdparty/pin_ik/tests/benchmark/arm6.urdf")
    target_link_libraries(${backend}_ik_workspace_survey PRIVATE ik_test_${backend} ik_test_parser)
    add_test(NAME ${backend}_ik_workspace_survey_smoke COMMAND ${backend}_ik_workspace_survey)
    set_tests_properties(${backend}_ik_workspace_survey_smoke PROPERTIES ENVIRONMENT
        "IK_SURVEY_MODEL=arm6;IK_SURVEY_KIND=warm;IK_SURVEY_COUNT=8;IK_DRIFT_OUTPUT=${CMAKE_BINARY_DIR}/${backend}_survey_smoke")
endforeach()
target_sources(pin_ik_workspace_survey PRIVATE fixed_configuration_ik.cpp)
add_test(NAME ajm_arm6_survey_smoke COMMAND pin_ik_workspace_survey)
set_tests_properties(ajm_arm6_survey_smoke PROPERTIES ENVIRONMENT
    "IK_DRIFT_BACKEND=anchored;IK_SURVEY_MODEL=arm6;IK_SURVEY_KIND=warm;IK_SURVEY_COUNT=8;IK_DRIFT_OUTPUT=${CMAKE_BINARY_DIR}/ajm_arm6_smoke")

target_sources(pin_ik_closed_path_test PRIVATE fixed_configuration_ik.cpp)
add_test(NAME fixed_configuration_closed_path COMMAND pin_ik_closed_path_test
    "--test-case=Standalone Talon closed path IK comparison")
set_tests_properties(fixed_configuration_closed_path PROPERTIES
    ENVIRONMENT "IK_DRIFT_BACKEND=anchored;IK_DRIFT_CYCLES=20;IK_DRIFT_OUTPUT=${CMAKE_BINARY_DIR}/anchored_smoke")
add_test(NAME fixed_configuration_faults COMMAND pin_ik_closed_path_test
    "--test-case=Fixed configuration IK rejects faults without committing output,Six-axis AJM uses zero extra constraints")
