cmake_minimum_required(VERSION 3.3.0)
project(serial_test)
find_package(cmake_modules)
set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -pg -O2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

set(SRC ${SRC}
    src/serial_comm_linux.cpp)

include_directories( ${includeDirs} )

add_executable (parani_rx src/main_parani_rx.cpp ${SRC})
add_executable (parani_tx src/main_parani_tx.cpp ${SRC})
add_executable (serial_rx src/main_serial_rx.cpp ${SRC})