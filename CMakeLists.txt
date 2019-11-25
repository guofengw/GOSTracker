cmake_minimum_required(VERSION 3.6)

project(GOSTracker)

# OpenCV
find_package(OpenCV REQUIRED)
include("cmake/opencv_helper.cmake")

add_subdirectory(3rdparty)
add_subdirectory(testbed)

file(GLOB SRCS src/*)
add_executable(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS} glad freeglut_static ${OPENGL_LIBRARIES} levmar)
target_include_directories(${PROJECT_NAME} PUBLIC src/.)

if(MSVC)
  set_target_properties(${PROJECT_NAME} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/data")
endif()