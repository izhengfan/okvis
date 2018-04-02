cmake_minimum_required(VERSION 2.8.3)
project(okvis_matcher)

# build the library
add_library(${PROJECT_NAME}
  src/DenseMatcher.cpp
  src/MatchingAlgorithm.cpp
  src/ThreadPool.cpp
  include/okvis/DenseMatcher.hpp
  include/okvis/MatchingAlgorithm.hpp
  include/okvis/ThreadPool.hpp
)

# and link it
target_link_libraries(${PROJECT_NAME}
  PUBLIC okvis_util
)

# installation if required
install(TARGETS ${PROJECT_NAME}
  EXPORT okvisTargets 
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/okvis" COMPONENT dev
)
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR} COMPONENT dev FILES_MATCHING PATTERN "*.hpp")

