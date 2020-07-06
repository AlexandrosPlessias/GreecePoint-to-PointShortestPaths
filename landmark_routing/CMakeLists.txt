project(LandmarkRouting)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS                 "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++11")

message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}")
message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS system program_options REQUIRED)

if ( PostgreSQL_TYPE_INCLUDE_DIR )
  else ( PostgreSQL_TYPE_INCLUDE_DIR)
    set ( PostgreSQL_TYPE_INCLUDE_DIR ${PostgreSQL_INCLUDE_DIR})
  endif ( PostgreSQL_TYPE_INCLUDE_DIR )
#set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/..")
find_package(PostgreSQL REQUIRED)

include_directories(/usr/include/postgresql/)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    add_executable(${PROJECT_NAME} main.cpp Graph.cpp DBMgmt.cpp Astar.cpp)
    target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} pq)
endif()
