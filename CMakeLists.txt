cmake_minimum_required(VERSION 2.8)

set(PRJ_NAME "backimagecoor")
project(${PRJ_NAME})

#cmake调用环境变量的方式使用$ENV{NAME}指令
include_directories($ENV{LIB_ROOT}/eigen3.2)
#backimagecoor
set(READTIFF_NAME "checkpmvspoints")
set(READTIFF_SOURCES
  "src/checkpmvspoint.cpp"
  )
add_executable(${READTIFF_NAME} ${READTIFF_SOURCES})
