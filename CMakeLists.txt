# Set required CMake standard
cmake_minimum_required(VERSION 3.15) 

# Set required C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set name, version, and language of the project
project(pybind_example VERSION 1.0.0 LANGUAGES CXX)

# Configure build
set(CMAKE_BUILD_TYPE Debug)
set(BUILD_PYTHON_BINDINGS ON CACHE BOOL "Build Python bindings")

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
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.13.6
)
FetchContent_MakeAvailable(pybind11)

# Find libraries
find_package(Threads REQUIRED)



#############################################################
# PYTHON BINDINGS
#############################################################
# Check if Python bindings should be built
if(NOT BUILD_PYTHON_BINDINGS)
    message(STATUS "NOT building Python bindings")
    return()
endif()
message(STATUS "Building Python bindings")

# Pybind11
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
set(PYBIND11_FINDPYTHON ON)
set(PYTHON_BINDINGS_EXECUTABLE ${PROJECT_NAME}_python)

# Define the module to be built with pybind11
pybind11_add_module(${PROJECT_NAME}
    src/logging.cpp
    src/main.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME}
    PRIVATE ${INCLUDE_DIR}
    PRIVATE ${plog_SOURCE_DIR}/include
    PRIVATE ${PYBIND11_INCLUDE_DIR}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    PRIVATE Threads::Threads
)

# Set compile options, enable warnings
target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:-Wall -O3>
    $<$<CXX_COMPILER_ID:MSVC>:/W3 /O3>
)

# Generate the .pyi file (stubs) after building the module
set(STUBS_DIR ${CMAKE_CURRENT_BINARY_DIR}/stubs)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E env
        PYTHONPATH=$<TARGET_FILE_DIR:${PROJECT_NAME}>
        LD_LIBRARY_PATH=$<TARGET_FILE_DIR:${PROJECT_NAME}>:$ENV{LD_LIBRARY_PATH}
        ${Python3_EXECUTABLE} -m pybind11_stubgen ${PROJECT_NAME} --output-dir=${STUBS_DIR}
    WORKING_DIRECTORY $<TARGET_FILE_DIR:${PROJECT_NAME}>
    COMMENT "Generating .pyi file for ${PROJECT_NAME}"
)

# Set up installation directory
install(TARGETS ${PROJECT_NAME} DESTINATION ${PROJECT_NAME})
install(FILES ${STUBS_DIR}/${PROJECT_NAME}.pyi DESTINATION ${PROJECT_NAME})
install(FILES python/__init__.py DESTINATION ${PROJECT_NAME})