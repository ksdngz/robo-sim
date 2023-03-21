cmake_minimum_required(VERSION 3.1)
#project(movingPendulum CXX)
project(movingPendulum)


set(SOURCES main.cpp mainMujoco.c)
set(INCLUDE_DIR ../../../mujoco/include/)
set(LIB_DIR ../../bin)



message(${SOURCES})
add_executable(moving-Pendulum 
main.cpp 
mainMujoco.c)
#add_executable(movingPendulum ${SOURCES})

target_include_directories(moving-Pendulum PRIVATE ${INCLUDE_DIR})
target_link_directories(moving-Pendulum PRIVATE ${LIB_DIR})
target_link_libraries(moving-Pendulum mujoco200 GL m glew libglfw.so.3)



