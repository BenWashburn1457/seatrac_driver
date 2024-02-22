# FRoSt seatrac_driver #

This is a driver for the Blueprint Subsea Seatrac USBL receiver.

It is a fork of the seatrac_driver written by Pierre Narvor (https://gitlab.ensta-bretagne.fr/narvorpi/seatrac_driver). This fork was made to add the DataSend, DataReceive, and DataError message types

## Installation

This is a standard cmake package. It is to be installed in a location pointed by
the CMAKE_PREFIX_PATH environment variable.

```
git clone git@bitbucket.org:frostlab/seatrac_driver.git && cd seatrac_driver

mkdir build && cd build

cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=<your_install_location> ..

make -j4 install
```

Make sure <your_install_location> is in the CMAKE_PREFIX_PATH environment
variable.

```
echo $CMAKE_PREFIX_PATH
```

If not, be sure to add it (for example with the following line :)

```
echo "export CMAKE_PREFIX_PATH=<your_install_location_full_path>:\$CMAKE_PREFIX_PATH" >> .bashrc
```

### Dependancies
* Boost: if you do not have boost on your system, you can install it with ```sudo apt-get install libboost-all-dev```

## Using in your C++ project

### CMake

To include this driver into your C++ project, link with it as you would with any
CMake package :

In your CMakeLists.txt :

```
find_package(seatrac_driver REQUIRED)

add_executable(your_executable ...)
target_link_libraries(your_executable PRIVATE
    seatrac_driver
)
```

And that's it ! (No "include_directories" or ${seatrac_driver_LIBRARIES}
shenanigans. Please use modern CMake.)

### In your code

You can receive data from the driver by subclassing the "SeatracDriver" class
and reimplementing the "on_receive" virtual method.


Your implementation should look like this :

```
#include <seatrac_driver/SeatracDriver.h>

class MySeatracDriver : public seatrac::SeatracDriver
{
    public:

    MySeatracDriver(const IoServicePtr& ioService,
                    const std::string& serialDevice = "/dev/ttyUSB0") :
        seatrac::SeatracDriver(ioService, serialDevice)
    {}

    protected:

    virtual void on_message(CID_E msgId, const std::vector<uint8_t>& msgData)
    {
        // your message handlers here.
    }
};
```