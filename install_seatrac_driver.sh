
mkdir build
cd build

cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="/seatrac_driver" ..

sudo make -j4 install

cd ..
