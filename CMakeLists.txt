# Set required CMake standard
cmake_minimum_required(VERSION 3.15) 

# Set required C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set name, version, and language of the project
project(safe_motion VERSION 1.0.0 LANGUAGES CXX)

# Configure build
set(CMAKE_BUILD_TYPE Debug)

# Define paths
set(INCLUDE_DIR include)
set(LIB_DIR ${CMAKE_SOURCE_DIR}/lib)
set(FETCHCONTENT_BASE_DIR ${LIB_DIR})

#############################################################
# LIBRARIES
#############################################################
include(FetchContent)
FetchContent_Declare(
    plog
    GIT_REPOSITORY https://github.com/SergiusTheBest/plog.git
    GIT_TAG 1.1.10
)
FetchContent_MakeAvailable(plog)

FetchContent_Declare(
    MeshcatCpp
    GIT_REPOSITORY https://github.com/ami-iit/meshcat-cpp.git
    GIT_TAG a84be7add7f344d61e615bee7f26e6a7d5444f2a
)
FetchContent_MakeAvailable(MeshcatCpp)


# Find libraries
find_package(Threads REQUIRED)
list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots")
find_package(pinocchio REQUIRED)


#############################################################
# CPP
#############################################################
add_executable(${PROJECT_NAME}_cpp
    src/logging.cpp
    src/main.cpp
    src/robot.cpp
    src/meshcat_utilities.cpp
    src/geometry.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME}_cpp
    PRIVATE ${INCLUDE_DIR}
    PRIVATE ${plog_SOURCE_DIR}/include
    PRIVATE ${PYBIND11_INCLUDE_DIR}
    PRIVATE ${pinocchio_SOURCE_DIR}/include
    PRIVATE ${MeshcatCpp_SOURCE_DIR}/include
)

# Link libraries
target_link_libraries(${PROJECT_NAME}_cpp
    PRIVATE Threads::Threads
    PRIVATE pinocchio::pinocchio
    PRIVATE MeshcatCpp::MeshcatCpp
)

# Set compile options, enable warnings
set(OPTIMIZATION_LEVEL 3)
target_compile_options(${PROJECT_NAME}_cpp PRIVATE
$<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:-Wall -O${OPTIMIZATION_LEVEL}>
$<$<CXX_COMPILER_ID:MSVC>:/W0 /O${OPTIMIZATION_LEVEL}>
)