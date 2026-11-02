# 保持上游子模块不变，复用 App 的 NLopt，不构建上游示例或安装目标。
set(ROCOS_PIN_IK_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../3rdparty/pin_ik")
if(NOT EXISTS "${ROCOS_PIN_IK_SOURCE_DIR}/src/pin_ik.cpp")
    message(FATAL_ERROR "pin_ik submodule is missing. Run: git submodule update --init 3rdparty/pin_ik")
endif()
add_library(pin_ik STATIC
    ${ROCOS_PIN_IK_SOURCE_DIR}/src/pinocchio_tl.cpp
    ${ROCOS_PIN_IK_SOURCE_DIR}/src/nlopt_ik.cpp
    ${ROCOS_PIN_IK_SOURCE_DIR}/src/pin_ik.cpp
    ${ROCOS_PIN_IK_SOURCE_DIR}/src/urdf.cpp)
set_target_properties(pin_ik PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(pin_ik PUBLIC cxx_std_17)
target_include_directories(pin_ik PUBLIC ${ROCOS_PIN_IK_SOURCE_DIR}/include)
target_link_libraries(pin_ik PUBLIC pinocchio::pinocchio Eigen3::Eigen Threads::Threads nlopt)
