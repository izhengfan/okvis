cmake_minimum_required(VERSION 2.8.11)

project(okvis_kinematics)

# require Eigen3
find_package( Eigen REQUIRED )
include_directories(${EIGEN_INCLUDE_DIR})

# build the library
add_library(${PROJECT_NAME} STATIC src/dependency-tracker.cc)

# and link it
target_link_libraries(${PROJECT_NAME}
  PUBLIC okvis_util
)

# installation if required
install(TARGETS ${PROJECT_NAME}
  EXPORT okvisTargets
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
)
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.hpp")

