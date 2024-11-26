# Set required CMake standard
cmake_minimum_required(VERSION 3.15) 

# Set required C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set name, version, and language of the project
project(pinocchio_utilities_example VERSION 1.0.0 LANGUAGES CXX)

# Configure build 
set(CMAKE_BUILD_TYPE Debug)
option(LOGGING "Print logs to the console" ON) 
set(OPTIMIZATION_LEVEL 3)

# Define paths
set(INCLUDE_DIR include)

#############################################################
# LIBRARIES
#############################################################
include(FetchContent)

if(${LOGGING})
FetchContent_Declare(
    plog
    GIT_REPOSITORY https://github.com/SergiusTheBest/plog.git
    GIT_TAG 1.1.10
)
FetchContent_MakeAvailable(plog)
endif()

FetchContent_Declare(
    MeshcatCpp
    GIT_REPOSITORY https://github.com/ami-iit/meshcat-cpp.git
    GIT_TAG a84be7add7f344d61e615bee7f26e6a7d5444f2a
)
FetchContent_MakeAvailable(MeshcatCpp)


# Find libraries
find_package(Threads REQUIRED)
list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots") # Default install path of Pinocchio
find_package(pinocchio REQUIRED)


add_executable(${PROJECT_NAME}
    src/example.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME}
    PRIVATE ${INCLUDE_DIR}
    PRIVATE ${plog_SOURCE_DIR}/include
    PRIVATE ${PYBIND11_INCLUDE_DIR}
    PRIVATE ${pinocchio_SOURCE_DIR}/include
    PRIVATE ${MeshcatCpp_SOURCE_DIR}/include
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    PRIVATE Threads::Threads
    PRIVATE pinocchio::pinocchio
    PRIVATE MeshcatCpp::MeshcatCpp
)

# Logging
if(${LOGGING})
    target_compile_definitions(${PROJECT_NAME} PRIVATE LOGGING)
endif()

# Set compile options, enable warnings
target_compile_options(${PROJECT_NAME} PRIVATE
$<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:-Wall -O${OPTIMIZATION_LEVEL}>
$<$<CXX_COMPILER_ID:MSVC>:/W0 /O${OPTIMIZATION_LEVEL}>
)

