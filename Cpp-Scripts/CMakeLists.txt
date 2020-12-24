cmake_minimum_required(VERSION 2.8)

# Give a name to the project.
project(cpp-nao-control)

#Vrep integration
add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -DDO_NOT_USE_SHARED_MEMORY)
include_directories(cpp-nao-control remoteApi)
include_directories(cpp-nao-control include)


# You need this to find the qiBuild CMake framework
find_package(qibuild)
find_package(qimodule)

#create executable

 qi_create_bin(cpp-nao-control
   SRC vrepsimulation.h vrepsimulation.cpp vrepsimulationmain.cpp
   DEPENDS BOOST QI
   )

#create lib
#qi_create_lib(cpp-nao-control
#  SRC vrepsimulation.h vrepsimulation.cpp vrepsimulationmain.cpp
#  DEPENDS BOOST QI
#  )
  # Tell CMake that mybroker depends on ALCOMMON and ALPROXIES
# This will set the libraries to link mybroker with,
# the include paths, and so on
qi_use_lib(cpp-nao-control BOOST)
