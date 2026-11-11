cmake_minimum_required(VERSION 3.15)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(librocos LANGUAGES CXX)
    enable_testing()
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()

find_package(Threads REQUIRED)
set(ROCOS_SOURCE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
add_library(librocos STATIC src/robot.cpp)
add_library(librocos::librocos ALIAS librocos)
set_target_properties(librocos PROPERTIES OUTPUT_NAME rocos POSITION_INDEPENDENT_CODE ON)
target_compile_features(librocos PUBLIC cxx_std_17)
target_include_directories(librocos PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE ${ROCOS_SOURCE_ROOT}/3rdparty/httplib ${ROCOS_SOURCE_ROOT}/3rdparty/json
            ${ROCOS_SOURCE_ROOT}/src)
target_link_libraries(librocos PRIVATE Threads::Threads)
target_compile_options(librocos PRIVATE -Wall -Wextra -Wpedantic)

option(LIBROCOS_BUILD_EXAMPLES "Build librocos joint position examples" ON)
option(LIBROCOS_BUILD_TESTS "Build librocos offline protocol tests" ON)
if(LIBROCOS_BUILD_EXAMPLES)
    foreach(example echo_robot_state generate_joint_position_motion
                    joint_point_to_point_motion generate_consecutive_motions communication_test
                    joint_6_7_sine)
        add_executable(librocos_${example} examples/${example}.cpp)
        target_link_libraries(librocos_${example} PRIVATE librocos)
    endforeach()
    find_package(yaml-cpp REQUIRED)
    target_include_directories(librocos_joint_6_7_sine PRIVATE
        ${ROCOS_SOURCE_ROOT}/3rdparty/httplib ${ROCOS_SOURCE_ROOT}/3rdparty/json
        ${ROCOS_SOURCE_ROOT}/src)
    target_link_libraries(librocos_joint_6_7_sine PRIVATE yaml-cpp tinyxml2 Threads::Threads)
endif()
if(LIBROCOS_BUILD_TESTS)
    # 在线集成程序需显式准备仿真或硬件，保持手动运行。
    add_executable(librocos_online_test ${ROCOS_SOURCE_ROOT}/test/librocos_online_test.cc)
    target_link_libraries(librocos_online_test PRIVATE librocos)
    add_executable(librocos_test ${ROCOS_SOURCE_ROOT}/test/librocos_test.cc)
    target_include_directories(librocos_test PRIVATE ${ROCOS_SOURCE_ROOT}
        ${ROCOS_SOURCE_ROOT}/3rdparty/httplib ${ROCOS_SOURCE_ROOT}/3rdparty/json)
    target_link_libraries(librocos_test PRIVATE librocos Threads::Threads)
    if(LIBROCOS_BUILD_EXAMPLES)
        target_compile_definitions(librocos_test PRIVATE
            LIBROCOS_EXAMPLE_DIR="$<TARGET_FILE_DIR:librocos_echo_robot_state>"
            LIBROCOS_SOURCE_ROOT="${ROCOS_SOURCE_ROOT}")
        target_link_libraries(librocos_test PRIVATE yaml-cpp)
        add_dependencies(librocos_test librocos_echo_robot_state
            librocos_generate_joint_position_motion librocos_joint_point_to_point_motion
            librocos_generate_consecutive_motions librocos_communication_test librocos_joint_6_7_sine)
    endif()
    add_test(NAME librocos_test COMMAND librocos_test)
    set_tests_properties(librocos_test PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
install(TARGETS librocos EXPORT librocosTargets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/librocos DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT librocosTargets NAMESPACE librocos:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/librocos)
configure_package_config_file(cmake/librocosConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/librocosConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/librocos)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/librocosConfig.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/librocos)
