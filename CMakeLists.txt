cmake_minimum_required(VERSION 3.5)
project(libmotioncapture)

# define some options
option(BUILD_PYTHON_BINDINGS "Generate Python Bindings" ON)
option(ENABLE_QUALISYS "Enable Qualisys" ON)
option(ENABLE_OPTITRACK "Enable Optitrack" ON)
option(ENABLE_VICON "Enable Vicon" ON)
option(ENABLE_VRPN "Enable VRPN" ON)
option(BUILD_EXAMPLE "Enable Example application" ON)

# Enable C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Boost 1.5 COMPONENTS system REQUIRED) # for optitrack
add_definitions(
  -DBOOST_DATE_TIME_NO_LIB
  -DBOOST_REGEX_NO_LIB
  -D_BUILD_STATIC_LIB
)

find_package(Eigen3 REQUIRED)

set(VICON_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/externalDependencies/vicon-datastream-sdk/)
set(QUALISYS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/externalDependencies/qualisys_cpp_sdk/)
set(VRPN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/externalDependencies/vrpn/)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

###########
## Build ##
###########

## Additional include folders
set(my_include_directories
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
set(my_link_directories)
set(my_files
  src/motioncapture.cpp
  src/testmocap.cpp
)

if (ENABLE_VICON)
  add_definitions(-DENABLE_VICON)
  add_subdirectory(externalDependencies/vicon-datastream-sdk)
  set(my_include_directories
    ${my_include_directories}
    ${VICON_SDK_DIR}/Vicon/CrossMarket/DataStream
  )
  set(my_files
    ${my_files}
    src/vicon.cpp
  )
  set(my_libraries
    ${my_libraries}
    ViconDataStreamSDK_CPP
  )
endif()

if (ENABLE_OPTITRACK)
  add_definitions(-DENABLE_OPTITRACK)
  set(my_include_directories
    ${my_include_directories}
  )
  set(my_files
    ${my_files}
    src/optitrack.cpp
  )
endif()

if (ENABLE_QUALISYS)
  add_definitions(-DENABLE_QUALISYS)
  add_subdirectory(externalDependencies/qualisys_cpp_sdk)
  set(my_include_directories
    ${my_include_directories}
    ${QUALISYS_DIR}
  )
  set(my_files
    ${my_files}
    src/qualisys.cpp
  )
  set(my_libraries
    ${my_libraries}
    qualisys_cpp_sdk
  )
endif()

if (ENABLE_VRPN)
  add_definitions(-DENABLE_VRPN)
  option(VRPN_BUILD_CLIENT_LIBRARY "" ON)
  add_subdirectory(externalDependencies/vrpn)
  set(my_include_directories
    ${my_include_directories}
    ${VRPN_DIR}
  )
  set(my_files
    ${my_files}
    src/vrpn.cpp
  )
  set(my_libraries
    ${my_libraries}
    ${VRPN_CLIENT_LIBRARY}
  )
endif()


include_directories(
  ${my_include_directories}
)

link_directories(
  ${my_link_directories}
)

## Declare a cpp library
add_library(libmotioncapture
  ${my_files}
)

## Specify libraries to link a library or executable target against
target_link_libraries(libmotioncapture
  ${my_libraries}
)
set_property(TARGET libmotioncapture PROPERTY POSITION_INDEPENDENT_CODE ON)
set(LIBMOTIONCAPTURE_LINK_DIR ${my_link_directories} CACHE STRING "link directories for libmotioncapture")

if (BUILD_PYTHON_BINDINGS)
  # Python bindings
  add_subdirectory(externalDependencies/pybind11)
  # find_package(Python COMPONENTS Interpreter Development)
  # find_package(pybind11 CONFIG)

  pybind11_add_module(motioncapture
    src/python_bindings.cpp
  )

  target_link_libraries(motioncapture
    PRIVATE
      libmotioncapture
  )
endif()

if (BUILD_EXAMPLE)
  add_executable(motioncapture_example
    examples/main.cpp
  )
  target_link_libraries(motioncapture_example
    libmotioncapture
  )

endif()