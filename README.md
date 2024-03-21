# FRoSt seatrac_driver #

This is a driver for the Blueprint Subsea Seatrac USBL receiver.

It is a fork of the seatrac_driver written by Pierre Narvor (https://gitlab.ensta-bretagne.fr/narvorpi/seatrac_driver). This fork was made to add the DataSend, DataReceive, and DataError message types

## Installation

This is a standard cmake package. It is to be installed in a location pointed by
the CMAKE_PREFIX_PATH environment variable.

```
git clone https://Clayton314@bitbucket.org/frostlab/seatrac_driver.git

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

### Dependancy Issues
The driver may not run as intended if these dependancies have not been previously installed:
* rtac_asio: if you get errors where the driver cannot find the librtac_asio.so 
  file, you may need to install rtac_asio separately. To install, follow the instructions at 
  https://github.com/pnarvor/rtac_asio. After installing, reinstall the seatrac driver.
* Boost: if you do not have boost on your system, you can install it with 
  ```sudo apt-get install libboost-all-dev```

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

### Blueprint Subsea Seatrac Developer Guide

This driver follows the guidlines found in the Seatrac Developer Guide (linked below).
The manual is a good reference for definitions of variables and types. In addition, it
defines other modem actions not yet included in this driver, which may be helpfull if 
you need different functionality for your project.

(https://www.seascapesubsea.com/downloads/Blueprint-Subsea-SeaTrac-Developer-Guide.pdf)

