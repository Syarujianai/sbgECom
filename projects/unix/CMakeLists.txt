cmake_minimum_required(VERSION 2.8)

project(sbgECom)
#specify this option when compiling by cmake instruction
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

#specify the search path for header file from ./src/ and ./common/
#and PROJECT_SOURCE_DIR is a cmake environment variable means - TOP-LEVEL(you are using cmake command)directory "projects/unix/"
#meanwhile, "/.." means - parent directory
include_directories(
	${PROJECT_SOURCE_DIR}/../../src/
	${PROJECT_SOURCE_DIR}/../../common/
	${PROJECT_SOURCE_DIR}/../../user/	
)

#all library.a and execute.o ouput in the same directory ./bin/,so we can ignore adding library directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../../bin/)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../../bin/)

#example:GLOB_RECURSE SRC/src/*.c - match all .c files in /src and subdirectories and assign to SRC list variable
file(GLOB_RECURSE SRC ${PROJECT_SOURCE_DIR}/../../src/*.c)
file(GLOB_RECURSE COMMON_SRC ${PROJECT_SOURCE_DIR}/../../common/*.c)
file(GLOB_RECURSE USER_SRC ${PROJECT_SOURCE_DIR}/../../user/*.cpp)

#
#	Exclude platform specific files
#
#REMOVE_ITEM(key word):delete some "${PROJECT_SOURCE_DIR}/../../common/interfaces/sbgInterfaceSerialWin.c") from ${COMMON_SRC}
if (NOT MSVC)
	list(REMOVE_ITEM COMMON_SRC "${PROJECT_SOURCE_DIR}/../../common/interfaces/sbgInterfaceSerialWin.c")
else ()
	list(REMOVE_ITEM COMMON_SRC "${PROJECT_SOURCE_DIR}/../../common/interfaces/sbgInterfaceSerialUnix.c")
endif()

#car/sbgcom compile as static library
add_library(sbgECom STATIC ${SRC} ${COMMON_SRC})
add_library(user STATIC ${USER_SRC})#......

add_executable(ellipseMinimal "${PROJECT_SOURCE_DIR}/../../examples/ellipseMinimal/src/ellipseMinimal.cpp" )
#ellipseMinimal link to sbgcom/user
target_link_libraries(ellipseMinimal sbgECom user)#user.....
#example:add_dependencies(target1 t2 t3) represent that target1 depend on t2 t3
add_dependencies(ellipseMinimal sbgECom user)

