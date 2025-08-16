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
set(SOURCES
    samples/simple_sim.cpp
#    samples/world_spline.cpp
#    samples/world_spline.hpp
)

add_executable(simple_sim ${SOURCES})

# GLFW manual include/lib
set(GLFW_ROOT "${CMAKE_SOURCE_DIR}/third_party/glfw")
include_directories(${GLFW_ROOT}/include)
target_link_libraries(simple_sim PRIVATE mujoco "${GLFW_ROOT}/lib/glfw3.lib")

# Add Eigen include directory
set(EIGEN_DIR "${CMAKE_SOURCE_DIR}/third_party/eigen")
include_directories(${EIGEN_DIR})
set(EIGEN_UNSUPPORTED_DIR "${CMAKE_SOURCE_DIR}/third_party/eigen/unsupported")
include_directories(${EIGEN_UNSUPPORTED_DIR})

# For Windows: copy DLL to output dir after build
if (WIN32)
    add_custom_command(TARGET simple_sim POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${MUJOCO_ROOT}/mujoco.dll"
            $<TARGET_FILE_DIR:simple_sim>)
endif()

if (WIN32)
    set(EMBED_PYTHON_DIR "${CMAKE_SOURCE_DIR}/third_party/python")
    if (EXISTS "${EMBED_PYTHON_DIR}/python310.dll")
        add_custom_command(TARGET simple_sim POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${EMBED_PYTHON_DIR}/python310.dll"
                $<TARGET_FILE_DIR:simple_sim>)
    endif()
endif()

# --- Python detection (try embedded python first, then system) ---
set(EMBED_PYTHON_DIR "${CMAKE_SOURCE_DIR}/third_party/python")
if (EXISTS "${EMBED_PYTHON_DIR}/python.exe")
    message(STATUS "Attempting to use embedded Python at ${EMBED_PYTHON_DIR}")
    # Try to find Python using embedded directory as hint (do not restrict to only this path yet)
    find_package(Python3 COMPONENTS Interpreter Development HINTS "${EMBED_PYTHON_DIR}" PATHS "${EMBED_PYTHON_DIR}")
    if (NOT Python3_FOUND)
        message(STATUS "Embedded Python not fully discoverable; falling back to system Python3 discovery")
        find_package(Python3 COMPONENTS Interpreter Development)
    endif()
else()
    find_package(Python3 COMPONENTS Interpreter Development)
endif()

if (Python3_FOUND)
    message(STATUS "Using Python3: ${Python3_EXECUTABLE}")
    target_include_directories(simple_sim PRIVATE ${Python3_INCLUDE_DIRS})
    target_link_libraries(simple_sim PRIVATE ${Python3_LIBRARIES})
else()
    message(WARNING "No embedded Python found and system Python3 not found. Python plotting will be disabled.")
endif()
