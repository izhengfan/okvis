git clone https://ceres-solver.googlesource.com/ceres-solver ceres
cd ceres
git checkout 7c57de5080c9f5a4f067e2d20b5f33bad5b1ade6
mkdir build
cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++11 -fPIC -mssse3" \
    -DCMAKE_INSTALL_PREFIX:PATH="~/.local/okvis/" \
    -DBUILD_EXAMPLES:BOOL=OFF \
    -DBUILD_TESTING:BOOL=OFF \
    -DBUILD_DOCUMENTATION:BOOL=OFF
make
make install
