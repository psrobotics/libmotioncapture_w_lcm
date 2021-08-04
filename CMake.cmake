cmake_minimum_required(VERSION 3.5)
project(libmotioncapture)

# define some options
option(BUILD_PYTHON_BINDINGS "Generate Python Bindings" ON)
option(ENABLE_QUALISYS "Enable Qualisys" OFF)
option(ENABLE_OPTITRACK "Enable Optitrack" ON)
option(ENABLE_VICON "Enable Vicon" OFF)
option(ENABLE_PHASESPACE "Enable PhaseSpace" OFF)
option(ENABLE_VRPN "Enable VRPN" OFF)

# Enable C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL REQUIRED)
set(VICON_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/externalDependencies/vicon-datastream-sdk/)
set(NATNET_DIR ${CMAKE_CURRENT_SOURCE_DIR}/externalDependencies/NatNetLinux/)
set(PHASESPACE_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/externalDependencies/phasespace_sdk/)
set(QUALISYS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/externalDependencies/qualisys_cpp_sdk/)

###########
## Build ##
###########

## Additional include folders
set(my_include_directories
  include
  ${PCL_INCLUDE_DIRS}
)
set(my_link_directories)
set(my_files
  src/motioncapture.cpp
  src/testmocap.cpp
)
set(my_libraries
  ${PCL_LIBRARIES}
)

if (ENABLE_VICON)
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
  set(my_include_directories
    ${my_include_directories}
    ${NATNET_DIR}/include
  )
  set(my_files
    ${my_files}
    src/optitrack.cpp
  )
endif()

if (ENABLE_PHASESPACE)
  set(my_include_directories
    ${my_include_directories}
    ${PHASESPACE_SDK_DIR}/include
  )
  set(my_link_directories
    ${my_link_directories}
    ${PHASESPACE_SDK_DIR}/lib64
  )
  set(my_files
    ${my_files}
    src/phasespace.cpp
  )
  set(my_libraries
    ${my_libraries}
    owlsock
    OWL
  )
endif()

if (ENABLE_QUALISYS)
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
  find_package(VRPN REQUIRED)
  set(my_include_directories
    ${my_include_directories}
    ${VRPN_INCLUDE_DIR}
  )
  set(my_files
    ${my_files}
    src/vrpn.cpp
  )
  set(my_libraries
    ${my_libraries}
    ${VRPN_LIBRARIES}
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