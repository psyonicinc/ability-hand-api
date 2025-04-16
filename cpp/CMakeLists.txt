project(AbilityHand)

cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 17)

# Check for Windows platform
if(WIN32)
    add_definitions(-DPLATFORM_WINDOWS)
    message(STATUS "Platform is Windows")

# Check for Linux platform
elseif(UNIX)
    add_definitions(-DPLATFORM_LINUX)
    message(STATUS "Platform is Linux")
endif()

add_subdirectory(ah_wrapper)
include_directories(${PROJECT_SOURCE_DIR}/ah_wrapper/include)

add_executable(main main.cpp)

target_link_libraries(main PRIVATE ah_wrapper)
