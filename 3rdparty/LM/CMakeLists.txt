set(PROJECT_NAME lm)

if(MSVC)
  if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/release/levmar.lib)
  else()
    set(LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/win64/mpbase.lib)
  endif()
elseif(ANDROID AND CMAKE_SYSTEM_PROCESSOR MATCHES "(ARM|arm)")
  if (CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/armeabi-v7a/libmpbase.so)
  else()
    set(LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm64-v8a/libmpbase.so)
  endif()
elseif(IOS)
  if(IOS_BITCODE)
    set(LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/ios_bitcode/libmpbase.a)
  else()
    set(LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/ios/libmpbase.a)
  endif()
elseif(UNIX AND CMAKE_SYSTEM_PROCESSOR MATCHES "(AMD64|X86|X64|amd64|x86|x64)")
  if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux32/mpbase.a)
  else()
    set(LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux64/mpbase.a)
  endif()
endif()

if(NOT LIB_PATH)
  message(WARNING "Unsupported platform for ${PROJECT_NAME}")
endif()

set(MPBASE "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH "")
set(MPBASE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/inc" CACHE PATH "")

if(LIB_PATH)
  set(MPBASE_LIB ${LIB_PATH} CACHE PATH "") # for display
  add_library(${PROJECT_NAME} STATIC IMPORTED GLOBAL)
  set_target_properties(${PROJECT_NAME} PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
    IMPORTED_IMPLIB ${LIB_PATH}
    IMPORTED_LOCATION ${LIB_PATH}
    )
else()
  add_library(${PROJECT_NAME} INTERFACE IMPORTED GLOBAL)
  set_target_properties(${PROJECT_NAME} PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
    )
endif()
