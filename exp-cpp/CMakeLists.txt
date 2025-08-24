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
    samples/constraint_path_planner.cpp
    samples/constraint_path_planner.hpp
    samples/path_planner.cpp
    samples/path_planner.hpp
)
add_executable(simple_sim ${SOURCES})

# GLFW manual include/lib
set(GLFW_ROOT "${CMAKE_SOURCE_DIR}/third_party/glfw")
include_directories(${GLFW_ROOT}/include)
target_link_libraries(simple_sim PRIVATE mujoco "${GLFW_ROOT}/lib/glfw3.lib")

# Eigen
set(EIGEN_DIR "${CMAKE_SOURCE_DIR}/third_party/eigen")
include_directories(${EIGEN_DIR})
set(EIGEN_UNSUPPORTED_DIR "${CMAKE_SOURCE_DIR}/third_party/eigen/unsupported")
include_directories(${EIGEN_UNSUPPORTED_DIR})

# boost
set(BOOST_ROOT       "${CMAKE_SOURCE_DIR}/third_party/boost_1_84_0/build")
set(boost_INCLUDE_DIRS "${BOOST_ROOT}/include/boost-1_84")
set(boost_INCLUDE_LIBS "${BOOST_ROOT}/lib")

# OMPL
set(ompl_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/third_party/ompl/install/include/ompl-1.8")
set(ompl_INCLUDE_LIBS "${CMAKE_SOURCE_DIR}/third_party/ompl/install/lib")
message(STATUS "ompl_INCLUDE_DIRS: ${ompl_INCLUDE_DIRS}")

target_include_directories(simple_sim
	PRIVATE 
	${ompl_INCLUDE_DIRS} 
	${boost_INCLUDE_DIRS} 
	${EIGEN_DIR} 
	${EIGEN_UNSUPPORTED_DIR} 
	${MUJOCO_ROOT}/include
)
target_link_directories(simple_sim
	PRIVATE 
	${ompl_INCLUDE_LIBS} 
	${boost_INCLUDE_LIBS} 
	${MUJOCO_ROOT}/lib
)
target_link_libraries(simple_sim 
	PRIVATE
	"${ompl_INCLUDE_LIBS}/ompl.lib"
#	For Debug
	"${boost_INCLUDE_LIBS}/libboost_serialization-vc143-mt-gd-1_84.lib"
	"${boost_INCLUDE_LIBS}/libboost_program_options-vc143-mt-gd-1_84.lib"
#	For Release
#	"${boost_INCLUDE_LIBS}/libboost_serialization-vc143-mt-1_84.lib"
#	"${boost_INCLUDE_LIBS}/libboost_program_options-vc143-mt-1_84.lib"
	mujoco
	"${GLFW_ROOT}/lib/glfw3.lib"
)
target_compile_features(simple_sim PRIVATE cxx_std_17)
if (MSVC)
    target_compile_definitions(simple_sim PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()


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

# --- Enforce usage of embedded Python only (no system fallback) ---
set(EMBED_PYTHON_DIR "${CMAKE_SOURCE_DIR}/third_party/python")

if (NOT EXISTS "${EMBED_PYTHON_DIR}/python.exe")
    message(FATAL_ERROR "Embedded Python not found at ${EMBED_PYTHON_DIR}. Run bootstrap.bat to install it.")
endif()

# Determine import library (support common names across minor versions)
set(_PY_CANDIDATE_LIBS
    "${EMBED_PYTHON_DIR}/python310.lib"
    "${EMBED_PYTHON_DIR}/python311.lib"
    "${EMBED_PYTHON_DIR}/python39.lib"
    "${EMBED_PYTHON_DIR}/libs/python310.lib"   # legacy layout safety
    "${EMBED_PYTHON_DIR}/libs/python39.lib"
)
set(_PY_LINK "")
foreach(lib ${_PY_CANDIDATE_LIBS})
    if (EXISTS "${lib}")
        set(_PY_LINK "${lib}")
        break()
    endif()
endforeach()

if (NOT _PY_LINK)
    message(FATAL_ERROR "No suitable Python import library (e.g., python310.lib or python39.lib) found in ${EMBED_PYTHON_DIR}. Ensure bootstrap copied the .lib file.")
endif()

# Header path: only support ${EMBED_PYTHON_DIR}/Include
set(_PY_INCLUDE_DIR "${EMBED_PYTHON_DIR}/Include")
if (EXISTS "${_PY_INCLUDE_DIR}/Python.h")
    target_include_directories(simple_sim PRIVATE "${_PY_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "Python headers not found at ${_PY_INCLUDE_DIR}. Run bootstrap.bat to fetch headers.")
endif()

target_link_libraries(simple_sim PRIVATE "${_PY_LINK}")

if (WIN32)
    # Suppress search for non-existent debug import libraries (common for embeddable distro)
    target_link_options(simple_sim PRIVATE 
        $<$<CONFIG:Debug>:/NODEFAULTLIB:python39_d.lib>
        $<$<CONFIG:Debug>:/NODEFAULTLIB:python310_d.lib>
        $<$<CONFIG:Debug>:/NODEFAULTLIB:python311_d.lib>
    )
endif()

message(STATUS "Embedded Python library: ${_PY_LINK}")
message(STATUS "Embedded Python includes: ${_PY_INCLUDE_DIR}")
