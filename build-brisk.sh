wget https://www.doc.ic.ac.uk/~sleutene/software/brisk-2.0.3.zip
unzip brisk-2.0.3.zip
cd brisk
mkdir build
cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++11 -fPIC -mssse3" \
    -DCMAKE_INSTALL_PREFIX:PATH="~/.local/okvis/"
make
make install
