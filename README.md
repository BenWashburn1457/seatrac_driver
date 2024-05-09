# FRoSt seatrac_driver

This is a driver for the Blueprint Subsea Seatrac USBL receiver.

It is a fork of the seatrac_driver written by Pierre Narvor 
https://gitlab.ensta-bretagne.fr/narvorpi/seatrac_driver

If this is your first time using this driver, I'd recommend starting with the
example code found in `seatrac_driver/examples`. Instructions for using the
examples is under the Examples heading in the README.

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

Alternatively, if you use `/lib/seatrac_driver` or `/usr/lib/seatrac_driver` as 
<your_install_location>, cmake can find the driver without setting CMAKE_PREFIX_PATH.

### Dependancy Issues
The driver has two principle dependancies:

* rtac_asio: while the driver is setup to find and download the library rtac_asio automatically
  sometimes there are errors. If cmake cannot find librtac_asio.so, you may need to install 
  rtac_asio separately. To install, follow the instructions at https://github.com/pnarvor/rtac_asio. 
  After installing, reinstall the seatrac driver.
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
#include <seatrac_driver/messages/Messages.h>

using namespace narval::seatrac;

class MySeatracDriver : public SeatracDriver
{
    public:

    MySeatracDriver(const IoServicePtr& ioService,
                    const std::string& serialDevice = "/dev/ttyUSB0") :
        SeatracDriver(ioService, serialDevice)
    {}

    protected:

    virtual void on_message(CID_E msgId, const std::vector<uint8_t>& msgData)
    {
        // your message handlers here.
    }
};
```

## Examples

This driver comes with 3 different examples detailing different features of the
driver. If this is your first time using this driver, these examples are the best place
to start.
### To run each example: 
1. Connect 2 beacons to your computer and place them together in water. 
2. Navigate to the examples folder `seatrac_driver/examples/<example_name>`
3. Build the example:
    ```
    mkdir build && cd build
    cmake ..
    make
    cd ..
    ```
    If you have installed the seatrac_driver and it is in /lib, /usr/lib, or 
    CMAKE_PREFIX_PATH, the example should use your installation. But you do 
    not need to install seatrac_driver beforehand to build an example. Each 
    example is setup to find and download the driver from the git
    repository (using FetchContent) if it cannot find an existing 
    seatrac_driver on your computer.
4. Run the example: `./build/<example_name> <serial_port>`
    The executable name is the same as example folder name.
    It takes one arguement - the serial port that the seatrac modem is 
    connected too (for example `/dev/ttyUSB0`).

### List of examples:
* ping_example: 
    Demonstrates the PING protocol. Sends a ping request to a nearby beacon with id 15.
    Once it receives a response, it prints out the data and sends another request to 
    the same beacon. Terminates once the enter key is pressed.
* data_send_example:
    Run data_send_example in two terminals connected to each beacon.
    Demonstrates the DAT protocol. You will be prompted to enter a string. It will 
    then send the string (up to 31 chars) to the other beacon, which will print 
    the data recieved in it's own terminal. Press enter without typing a message
    to close the program.
* calibration_example:
    Performs two example calibration sequences for the accelerometer and the 
    magnetometer. In this example, the calibration settings are only saved to RAM
    and will not override the settings already set on the beacon.
    To learn more about how to calibrate the beacon, read page 19 in the seatrac user guide:
    https://www.blueprintsubsea.com/downloads/seatrac/UM-140-P00918-03.pdf#page=19 


## Interfacing with the seatrac beacon
### Sending Commands to the beacon
You can send a command to the beacon in three steps: define the command's struct,
fill in the struct fields, and pass the struct to the `SeatracDriver::send` function.
In ping_example, a command is sent to send a ping request to another beacon. That is
accomplished with this method:
```
//lines 17-23 of seatrac_driver/examples/ping_example/src/ping_example.cpp with additional comments
17  void ping_beacon(BID_E target, AMSGTYPE_E pingType = MSG_REQU) {
18      messages::PingSend::Request req;
19      req.target   = target;    //target = BEACON_ID_15
20      req.pingType = pingType;  //pingType = MSG_REQU
21
22      this->send(sizeof(req), (const uint8_t*)&req); //'this' is of type SeatracDriver
23  }
```
Explaination:

* Line 18: Declares a struct of type `PingSend::Request`. 
* Line 19: Fills in the `target` field. This field indicates which beacon to send the acoustic message to.
    In this case it is `BEACON_ID_15`.
* Line 20: Fills in the `pingType` field. `MSG_REQU` indicates that our beacon should request a response
    from the other beacon. The `U` in `MSG_REQU` indicates that this response should include usbl information.
* Line 22: Sends the message to the beacon over a serial connection. `SeatracDriver::send` takes as input 
    a pointer to the raw bytes of the command being sent, so it is necessary recast the command struct.

### Decoding messages from the beacon
Decoding messages is very similar to sending one. All message decoding happens in the 
`SeatracDriver::on_message` method. `on_message` has two arguements: 

* `CID_E msgId`: This indicates the type of message that was received so you can know what struct to use to decode it.
* `const std::vector<uint8_t>& msgData`: the raw bytes of the message recieved.

To decode, make a switch statement on `msgId`, then add a case for your message. 
```
// ping_example CID_PING_RESP inside the MyDriver::on_message method
28  switch(msgId) {
... 
33      case CID_PING_RESP: {
34          messages::PingResp response;       //struct that contains response fields
35          response = data;                    //operator overload fills in response struct with correct data
36          std::cout << response << std::endl; //operator overload prints out response data
...
41      } break;
```
Explaination:

* Line 28: 


## Seatrac Support Website
https://www.blueprintsubsea.com/seatrac/support
This is the link to the latest seatrac beacon user support material.
It includes a link to the Seatrac User Guide (referenced in the calibration) and the Seatrac 