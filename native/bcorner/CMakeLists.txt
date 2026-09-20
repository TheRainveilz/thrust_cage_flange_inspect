cmake_minimum_required(VERSION 3.16)
project(bcorner_impl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MSVC 编译参数，放在find_package前面！
if(MSVC)
    # ✅ 修复：动态运行库正确写法
    set(CMAKE_MSVC_RUNTIME_LIBRARY "$<$<CONFIG:Debug>:MultiThreadedDebugDLL>$<$<NOT:$<CONFIG:Debug>>:MultiThreadedDLL>")
    add_compile_definitions(_USE_MATH_DEFINES)
    add_compile_options(/utf-8)
endif()

# pybind11
set(pybind11_DIR "I:/zq.zip/zq/.venv/Lib/site-packages/pybind11/share/cmake/pybind11")
find_package(pybind11 REQUIRED)

# OpenCV
set(OpenCV_DIR "D:/Software/ExploitationEnvironment/OpenCV/build")
find_package(OpenCV REQUIRED)

# 生成pybind模块，链接opencv
pybind11_add_module(bcorner b_corner.cpp)
target_link_libraries(bcorner PRIVATE ${OpenCV_LIBS})
