cmake_minimum_required(VERSION 2.8)

set(PRJ_NAME "backimagecoor")
project(${PRJ_NAME})

include_directories("F:/programs/lib/3rdparty/shared/Visual_Studio_10_Win64/eigen/inc")
#backimagecoor
set(READTIFF_NAME "checkpmvspoints")
set(READTIFF_SOURCES
  "src/checkpmvspoint.cpp"
  )
add_executable(${READTIFF_NAME} ${READTIFF_SOURCES})
