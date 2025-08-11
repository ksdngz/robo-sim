cmake_minimum_required(VERSION 3.10)
project(mujoco_sim_sample)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MuJoCo root directory (change if needed)
set(MUJOCO_ROOT "${CMAKE_SOURCE_DIR}/mujoco-3.3.5")

include_directories(${MUJOCO_ROOT}/include)
link_directories(${MUJOCO_ROOT}/lib)

# Sample source
add_executable(simple_sim samples/simple_sim.cpp)

# Link MuJoCo library (update name if needed)
target_link_libraries(simple_sim mujoco)

# For Windows: copy DLL to output dir after build
if (WIN32)
    add_custom_command(TARGET simple_sim POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${MUJOCO_ROOT}/mujoco.dll"
            $<TARGET_FILE_DIR:simple_sim>)
endif()
