project(glad)

file(GLOB_RECURSE SRCS include/*.h src/*.c)
add_library(${PROJECT_NAME} SHARED ${SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_compile_definitions(${PROJECT_NAME} PUBLIC GLAD_GLAPI_EXPORT PRIVATE GLAD_GLAPI_EXPORT_BUILD)

find_package(OpenGL REQUIRED)
target_include_directories(${PROJECT_NAME} INTERFACE ${OPENGL_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} INTERFACE ${OPENGL_LIBRARIES})
if(UNIX)
  target_link_libraries(${PROJECT_NAME} INTERFACE dl)
endif()
