mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX:PATH="~/.local/okvis/"
make
make install
