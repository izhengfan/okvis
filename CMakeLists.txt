cmake_minimum_required(VERSION 2.8.11)
message("### use cmake version " ${CMAKE_VERSION})

project(okvis)
# The version number.
set(OKVIS_MAJOR_VERSION 1)
set(OKVIS_MINOR_VERSION 1)
set(OKVIS_PATCH_VERSION 3)
set(OKVIS_VERSION
  ${OKVIS_MAJOR_VERSION}.${OKVIS_MINOR_VERSION}.${OKVIS_PATCH_VERSION})

set(CMAKE_INSTALL_PREFIX $ENV{HOME}/.local/okvis-master)

# options for build configuration
option (USE_SYSTEM_BRISK
  "Use brisk via find_package rather than downloading it as part of okvis" OFF) 
option (USE_SYSTEM_CERES
  "Use ceres via find_package rather than downloading it as part of okvis" OFF) 
option (BUILD_APPS
  "Builds a demo app (which requires boost)" ON) 
option (BUILD_TESTS
  "Builds all gtests" OFF) 
SET(N_CORES 2 CACHE STRING "Using N number of cores for parallel build")

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
  "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/CMake/okvis)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# make sure we use Release and warn otherwise
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()
if (NOT "${CMAKE_BUILD_TYPE}" STREQUAL "Release")
  message(WARNING "CMAKE_BUILD_TYPE not set to 'Release'. Performance may be terrible.")
else()
  message(STATUS "Building with build type '${CMAKE_BUILD_TYPE}'")
endif()

if(APPLE)
  # The clang compiler (on osx) is somehow much more strict
  # than the compilers on ubuntu and so this does not seem
  # possible on OSX just yet.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++11 -fPIC")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++11 -fPIC")
endif()

if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "arm*")
  message(STATUS "ARM processor detected, will attempt to use NEON.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon")
else()
  message(STATUS "Assuming SSE instructions available.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mssse3")
endif()

get_directory_property(defs COMPILE_DEFINITIONS)

# EXTERNAL DEPENDENCIES
include(ExternalProject)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include_directories(${CMAKE_BINARY_DIR}/include)
link_directories(${CMAKE_BINARY_DIR}/lib)
include_directories(${CMAKE_INSTALL_PREFIX}/include)
link_directories(${CMAKE_INSTALL_PREFIX}/lib)

# require Eigen
find_package( Eigen REQUIRED )
include_directories(${EIGEN_INCLUDE_DIR}) 

# libvisensor
find_package(VISensor)

if(VISENSORDRIVER_FOUND)
  add_definitions(-DHAVE_LIBVISENSOR)
  set(LIBVISENSOR TRUE)
  message(STATUS "Found libvisensor. Setting HAVE_LIBVISENSOR flag.")
  include_directories(${VISensorDriver_INCLUDE_DIR})
  # install find script
  install(FILES ${PROJECT_SOURCE_DIR}/cmake/FindVISensor.cmake
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/CMake)
else()
  message(STATUS "libvisensor not found")
endif()



# glog
find_package(Glog REQUIRED)
include_directories(BEFORE ${GLOG_INCLUDE_DIRS}) 

# brisk
find_package(OpenCV REQUIRED)
find_package(brisk 2 REQUIRED
    PATHS $ENV{HOME}/.local/okvis-master/lib/CMake/brisk NO_DEFAULT_PATH)
include_directories(${BRISK_INCLUDE_DIRS})
find_package( Ceres 1.9 REQUIRED
    PATHS $ENV{HOME}/.local/okvis-master/share/Ceres NO_DEFAULT_PATH)
include_directories(${CERES_INCLUDE_DIRS})
link_directories($ENV{HOME}/.local/okvis-master/lib/)
set(OpenGV_LIBRARIES opengv)


if(${BUILD_TESTS})
  # gmock
  # http://stackoverflow.com/questions/9689183/cmake-googletest
  ExternalProject_Add(
    googlemock
    URL https://googlemock.googlecode.com/files/gmock-1.7.0.zip
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gmock
    CMAKE_ARGS -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS} -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
    BUILD_COMMAND make -j${N_CORES}
    INSTALL_COMMAND ""
    )
  ExternalProject_Get_Property(googlemock source_dir)
  set(GMOCK_INCLUDE_DIR ${source_dir}/include)
  set(GTEST_INCLUDE_DIR ${source_dir}/gtest/include)
  include_directories(BEFORE ${GMOCK_INCLUDE_DIR})
  include_directories(BEFORE ${GTEST_INCLUDE_DIR})
  ExternalProject_Get_Property(googlemock binary_dir)
  set(GMOCK_LIBRARY_PATH ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}gmock.a)
  set(GMOCK_LIBRARY gmock)
  add_library(${GMOCK_LIBRARY} UNKNOWN IMPORTED)
  set_property(TARGET ${GMOCK_LIBRARY} PROPERTY IMPORTED_LOCATION
    ${GMOCK_LIBRARY_PATH} )
  add_dependencies(${GMOCK_LIBRARY} googlemock)
  set(GTEST_LIBRARY_PATH ${binary_dir}/gtest/${CMAKE_FIND_LIBRARY_PREFIXES}gtest.a)
  set(GTEST_LIBRARY gtestlocal)
  add_library(${GTEST_LIBRARY} UNKNOWN IMPORTED)
  set_property(TARGET ${GTEST_LIBRARY} PROPERTY IMPORTED_LOCATION
    ${GTEST_LIBRARY_PATH})
  add_dependencies(${GTEST_LIBRARY} googlemock)
endif()

# BUILD LOCAL DEPENDENCIES
include_directories(okvis_util/include)
add_subdirectory(okvis_util)

include_directories(okvis_kinematics/include)
add_subdirectory(okvis_kinematics)
add_dependencies(okvis_kinematics okvis_util)

include_directories(okvis_time/include)
add_subdirectory(okvis_time)
add_dependencies(okvis_time okvis_util)

include_directories(okvis_cv/include)
add_subdirectory(okvis_cv)
add_dependencies(okvis_cv brisk okvis_util)

include_directories(okvis_common/include)
add_subdirectory(okvis_common)

include_directories(okvis_ceres/include)
add_subdirectory(okvis_ceres)
add_dependencies(okvis_ceres ceres okvis_util)

include_directories(okvis_timing/include)
add_subdirectory(okvis_timing)

include_directories(okvis_matcher/include)
add_subdirectory(okvis_matcher)
add_dependencies(okvis_matcher okvis_util)

include_directories(okvis_frontend/include)
add_subdirectory(okvis_frontend)
add_dependencies(okvis_frontend opengv okvis_util)

include_directories(okvis_multisensor_processing/include)
add_subdirectory(okvis_multisensor_processing)
add_dependencies(okvis_multisensor_processing okvis_util)

set(OKVIS_CORE_LIBS
  okvis_util
  okvis_kinematics
  okvis_time
  okvis_cv
  okvis_common
  okvis_ceres
  okvis_timing
  okvis_matcher
  okvis_frontend
  okvis_multisensor_processing
  )
# also build the apps
if(BUILD_APPS)
  FIND_PACKAGE(Boost COMPONENTS filesystem system REQUIRED)
  include_directories(${Boost_INCLUDE_DIRS}) 
  add_executable(okvis_app_synchronous okvis_apps/src/okvis_app_synchronous.cpp)
  target_link_libraries(okvis_app_synchronous 
    ${OKVIS_CORE_LIBS}
    pthread 
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    brisk agast opengv
    )
  add_executable(okvis_app_synchronous_upenn okvis_apps/src/okvis_app_synchronous_upenn.cpp)
  target_link_libraries(okvis_app_synchronous_upenn
    ${OKVIS_CORE_LIBS}
    pthread
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    brisk agast opengv
    )
  add_executable(okvis_app_synchronous_kitti okvis_apps/src/okvis_app_synchronous_kitti.cpp)
  target_link_libraries(okvis_app_synchronous_kitti
    ${OKVIS_CORE_LIBS}
    pthread
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    brisk agast opengv
    )
  add_executable(okvis_app_synchronous_szri okvis_apps/src/okvis_app_synchronous_szri.cpp)
  target_link_libraries(okvis_app_synchronous_szri
    ${OKVIS_CORE_LIBS}
    pthread
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    brisk agast opengv
    )
  if(${VISENSORDRIVER_FOUND})
    target_link_libraries(okvis_app_synchronous ${VISensorDriver_LIBRARY})
  endif()
  install(TARGETS okvis_app_synchronous
    # IMPORTANT: Add the executable to the "export-set"
    EXPORT okvisTargets
    RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)
endif()

# installation is invoked in the individual modules...
export (TARGETS 
  okvis_app_synchronous
  okvis_util
  okvis_kinematics
  okvis_time
  okvis_cv 
  okvis_common
  okvis_ceres
  okvis_timing
  okvis_matcher
  okvis_frontend 
  okvis_multisensor_processing 
  FILE "${PROJECT_BINARY_DIR}/okvisTargets.cmake")
export ( PACKAGE okvis )

# Create the okvisConfig.cmake and okvisConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
  "${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS 
  "${PROJECT_SOURCE_DIR}/okvis_util/include"
  "${PROJECT_SOURCE_DIR}/okvis_kinematics/include"
  "${PROJECT_SOURCE_DIR}/okvis_time/include"
  "${PROJECT_SOURCE_DIR}/okvis_cv/include"
  "${PROJECT_SOURCE_DIR}/okvis_common/include"
  "${PROJECT_SOURCE_DIR}/okvis_ceres/include"
  "${PROJECT_SOURCE_DIR}/okvis_timing/include"
  "${PROJECT_SOURCE_DIR}/okvis_matcher/include"
  "${PROJECT_SOURCE_DIR}/okvis_frontend/include"
  "${PROJECT_SOURCE_DIR}/okvis_multisensor_processing/include"
  "${PROJECT_BINARY_DIR}"
  )
configure_file(cmake/okvisConfig.cmake.in
  "${PROJECT_BINARY_DIR}/okvisConfig.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${OKVIS_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(cmake/okvisConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/okvisConfig.cmake" @ONLY)
# ... for both
configure_file(cmake/okvisConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/okvisConfigVersion.cmake" @ONLY)

# Install the okvisConfig.cmake and okvisConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/okvisConfig.cmake"
  "${PROJECT_BINARY_DIR}/okvisConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}")

# Install the export set for use with the install-tree
install(EXPORT okvisTargets DESTINATION
  "${INSTALL_CMAKE_DIR}")
