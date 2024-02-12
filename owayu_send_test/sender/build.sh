
export CMAKE_PREFIX_PATH=/seatrac_driver:\$CMAKE_PREFIX_PATH

mkdir build
cd build

cmake ..
make

cd ..
