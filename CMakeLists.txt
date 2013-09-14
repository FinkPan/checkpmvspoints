cmake_minimum_required(VERSION 2.8)

project(checkpmvspoints)
#cmake调用环境变量的方式使用$ENV{NAME}指令
include_directories($ENV{LIB_ROOT}/eigen3.2)
include_directories(${PROJECT_SOURCE_DIR}/include)
#class checkpmvspoint
file(GLOB lib_srcs src/*.*)
file(GLOB lib_hdrs include/*.*)
#main
file(GLOB APP_SRC main/*.*)

source_group("INCLUDE" FILES ${lib_hdrs} )
source_group("SRC" FILES ${lib_srcs})

add_library(checkpmvspoints STATIC ${lib_hdrs} ${lib_srcs})

add_executable(main ${APP_SRC})
target_link_libraries(main checkpmvspoints)
