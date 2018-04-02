git clone https://github.com/laurentkneip/opengv.git
cd opengv
mkdir build
cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++11 -fPIC -mssse3" \
    -DCMAKE_INSTALL_PREFIX:PATH="~/.local/okvis/"
make
make install
