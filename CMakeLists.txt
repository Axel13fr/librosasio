cmake_minimum_required(VERSION 3.1)
project(librosasio)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Werror=return-type)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES librosasio
    CATKIN_DEPENDS roscpp
    DEPENDS Boost)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(librosasio SHARED src/asio_callbackqueue.cpp include/librosasio/asio_callbackqueue.h)
target_link_libraries(librosasio ${catkin_LIBRARIES} Boost::system)

add_executable(testnode examples/timers/main.cpp examples/timers/node.cpp)
target_link_libraries(testnode librosasio ${catkin_LIBRARIES} Boost::system)

## Mark executables and/or libraries for installation
install(TARGETS librosasio testnode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
