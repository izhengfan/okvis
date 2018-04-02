cmake_minimum_required(VERSION 2.8.11)
project(okvis_ceres)

# require Eigen
find_package( Eigen REQUIRED )
include_directories(${EIGEN_INCLUDE_DIR})

find_package(SuiteSparse REQUIRED QUIET) #will be needed in any case...
find_package( Ceres 1.9 REQUIRED
    PATHS $ENV{HOME}/.local/okvis/share/Ceres/ NO_DEFAULT_PATH)
include_directories(BEFORE ${CERES_INCLUDE_DIRS})
message(STATUS "Using system Ceres.")
find_package(OpenMP QUIET)
add_definitions(-DUSE_OPENMP)
add_library(${PROJECT_NAME}
  src/PoseParameterBlock.cpp
  src/SpeedAndBiasParameterBlock.cpp
  src/HomogeneousPointParameterBlock.cpp
  src/HomogeneousPointLocalParameterization.cpp
  src/PoseLocalParameterization.cpp
  src/ImuError.cpp
  src/PoseError.cpp
  src/RelativePoseError.cpp
  src/SpeedAndBiasError.cpp
  src/IdProvider.cpp
  src/Map.cpp
  src/MarginalizationError.cpp
  src/HomogeneousPointError.cpp
  src/Estimator.cpp
  src/LocalParamizationAdditionalInterfaces.cpp
  include/okvis/Estimator.hpp
  include/okvis/ceres/CeresIterationCallback.hpp
)

# and link it
target_link_libraries(${PROJECT_NAME}
  PUBLIC okvis_util
  PUBLIC okvis_cv
  PUBLIC okvis_common
  ${CERES_LIBRARIES}
  gomp
  PRIVATE ${OpenCV_LIBRARIES}
)

# installation if required
install(TARGETS ${PROJECT_NAME}
  EXPORT okvisTargets
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
)
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.hpp")
