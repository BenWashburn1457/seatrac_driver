# FRoSt seatrac_driver

This is a c++ driver for the Blueprint Subsea Seatrac USBL receiver.

It is a fork of the seatrac_driver written by Pierre Narvor:
https://gitlab.ensta-bretagne.fr/narvorpi/seatrac_driver.

If this is your first time using this driver, start with the
example code found in `seatrac_driver/examples`. Instructions for using the
examples are under the Examples heading in the README.

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
<your_install_location>, cmake can find the driver without CMAKE_PREFIX_PATH.

### Dependency Issues
The driver has two principle dependencies:

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
and reimplementing the "on_message" virtual method.


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

This driver comes with 4 different examples detailing different features of the
driver. For new users, these examples are the best place to start.

### List of examples:
* ping_example:
	Demonstrates the PING protocol. Sends a ping request to a nearby beacon with id 15.
	Once it receives a response, it prints out the data and sends another request to
	the same beacon. Terminates once the enter key is pressed.
* data_send_example:
	Run data_send_example in two terminals connected to each beacon.
	Demonstrates the DAT protocol. You will be prompted to enter a string. It will
	then send the string (up to 31 chars) to the other beacon, which will print
	the data received in its own terminal. Press enter without typing a message
	to close the program.
* calibration_example:
	Walks the user through a calibration procedure for either the magnetometer or accelerometer.
	You can run either a full calibration - which permanently overrides pre-existing calibration values -
	or a dry run - which only saves the values to RAM, leaving permanent memory untouched.
	To learn more about how to calibrate the beacon, read in the
	[Seatrac Beacon User Guide, page 19](https://www.blueprintsubsea.com/downloads/seatrac/UM-140-P00918-03.pdf#page=19).
* ros2_example:
	A ros2 node built for the humble distribution that publishes ping or data messages
	received by the beacon and subscribes to ping or data messages to send to other
	beacons.

### To run ping_example, data_send_example, or calibration_example:

1. Connect 2 beacons to your computer and place them together in water.
2. Navigate to the example's folder `seatrac_driver/examples/<example_name>`
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
	example is setup to find and download the driver from this git
	repository (using FetchContent) if it cannot find an existing
	seatrac_driver on your computer.
4. Run the example: `sudo ./build/<example_name> <serial_port>`
	The executable name is the same as the example folder name.
	It takes one argument - the serial port that the seatrac modem is
	connected too (for example `/dev/ttyUSB0`).
    
### To run ros2_example:

1. In a terminal with admin permissions, navigate to `seatrac_driver/examples/ros2_example`
2. Build the example and source local setup: `colcon build && source ./install/setup.bash`
3. Run: `ros2 run seatrac modem`

## Interfacing with the seatrac beacon
The seatrac_driver interfaces with the beacon through a serial connection. Messages are sent both ways
as a series of bytes. Messages sent to the beacon are commands. Messages received from the beacon
are responses.

The seatrac serial interface is defined in the [Seatrac Developer User Guide](https://www.blueprintsubsea.com/downloads/seatrac/UM-140-D00221-07.pdf).
The implementation of seatrac_driver closely matches the Developer Guide.

The first byte of each serial message (after the synchronization character) is the Command Identification Code,
defined in the [CID_E](https://www.blueprintsubsea.com/downloads/seatrac/UM-140-D00221-07.pdf#page=40) enumeration.
The following byte fields are determined by a unique structure for the CID_E. Each CID_E has up to two structures
defined for it, a command and a response.

For example, [CID_PING_SEND](https://www.blueprintsubsea.com/downloads/seatrac/UM-140-D00221-07.pdf#page=111)
has both a command and response format. If CID_PING_SEND is used as the first byte of a command sent to the
beacon, its second field would be DEST_ID and the third would be MSG_TYPE. If CID_PING_SEND is the first
byte of a response from the beacon, its second byte would be STATUS and the third is BEACON_ID.

In seatrac_driver, each message is defined as a struct. The first field of every message struct is
`static const CID_E Identifier`. This is set to the associated CID code. The remaining fields can be filled in
to form the complete message.

In addition to message structs, there are also field structs. Field structs are members of message structs.
They do not have a CID_E Identifier field.

#### Locations of Seatrac Structs and Enums in seatrac_driver

* Seatrac Enums are defined in [SeatracEnums.h](https://bitbucket.org/frostlab/seatrac_driver/src/main/include/seatrac_driver/SeatracEnums.h)
* Field Structs are defined in [SeatracTypes.h](https://bitbucket.org/frostlab/seatrac_driver/src/main/include/seatrac_driver/SeatracTypes.h)
* Message Structs are found in the folder [include/seatrac_driver/messages](https://bitbucket.org/frostlab/seatrac_driver/src/main/include/seatrac_driver/messages/).

In general, since the `CID_<MSG_TYPE>` symbols are already used in the CID_E enum in SeatracEnums.h,
the message struct names are written in CamelCase without the CID prefix. For example, the struct for
`CID_PING_RESP` is called [`PingResp`](https://bitbucket.org/frostlab/seatrac_driver/src/main/include/seatrac_driver/messages/PingResp.h#PingResp.h-9:22).

For **CIDs that have command and response** formats defined, the response struct is called `<MsgType>` and the command struct is called `<MsgType>::Request`. For `CID_PING_SEND`,
[`PingSend`](https://bitbucket.org/frostlab/seatrac_driver/src/main/include/seatrac_driver/messages/PingSend.h#PingSend.h-9:23)
is the struct for the response returned from the beacon, and
[`PingSend::Request`](https://bitbucket.org/frostlab/seatrac_driver/src/main/include/seatrac_driver/messages/PingSend.h#PingSend.h-14:18)
is the struct for the command sent to the beacon to send a ping.


## Sending Commands to the beacon

You can send a command to the beacon in three steps: define the command's struct,
fill in the struct fields, and pass it to one of the send functions of SeatracDriver.

In ping_example, a command is sent to send a ping request to another beacon. That is
accomplished with this method:

[ping_beacon in ping_example.cpp](https://bitbucket.org/frostlab/seatrac_driver/src/main/examples/ping_example/src/ping_example.cpp#ping_example.cpp-17:23)
```
//extra comments added for clarification
17  void ping_beacon(BID_E target, AMSGTYPE_E pingType = MSG_REQU) {
18  	messages::PingSend::Request req;
19  	req.target   = target;	//target = BEACON_ID_15
20  	req.pingType = pingType;  //pingType = MSG_REQU
21
22  	this->send(sizeof(req), (const uint8_t*)&req); //'this' is of type SeatracDriver
23  }
```

Explanation:

* Line 18: Declares a struct of type `PingSend::Request`, the command message for `CID_PING_SEND`.
* Line 19: Fills in the `target` field. This field indicates which beacon to send the acoustic message to.
	In this case it is `BEACON_ID_15`.
* Line 20: Fills in the `pingType` field. [`MSG_REQU`](https://www.seascapesubsea.com/downloads/Blueprint-Subsea-SeaTrac-Developer-Guide.pdf#page=31)
	indicates that our beacon should request a response from the other beacon. The `U` in `MSG_REQU`
	indicates that this response should include usbl information.
* Line 22: Sends the message to the beacon over the serial connection. `SeatracDriver::send` takes as input
	a pointer to the raw bytes of the command being sent, so it is necessary to recast the command struct.

seatrac_driver has two functions you can use to send data:

* `void SeatracDriver::send(size_t size, const uint8_t* data)` ---
	Send a command asynchronously to the beacon without waiting for a response.
	Takes two arguments:
	* `size` : The number of bytes in the command
	* `data` : A pointer to the command being sent
    
* `bool SeatracDriver::send_request(unsigned int cmdSize, const uint8_t* cmdData, T* respData, int64_t timeout)` ---
	Sends a command synchronously to the modem and waits for a response from the modem. It returns true if
	the response was successfully received and false otherwise.
	Takes four arguments:
	* `cmdSize`  : The number of bytes in the command
	* `cmdData`  : A pointer to the command being sent
	* `respData` : A struct that will be filled in with the response data from the beacon
	* `timeout`  : How long to wait for a response from the beacon

## Decoding messages from the beacon
Decoding a message is similar to sending one. All messages received
from the beacon result in a call to `SeatracDriver::on_message`. `on_message` has two arguments:

* `CID_E msgId`: This indicates the type of message that was received so you can know what struct to use to decode it.
* `const std::vector<uint8_t>& msgData`: the raw bytes of the message received.

ping_example also decodes and prints a ping response message received from the other beacon:

[Ping response message handler in ping_example.cpp](https://bitbucket.org/frostlab/seatrac_driver/src/main/examples/ping_example/src/ping_example.cpp#ping_example.cpp-28,33:41)
```
28  switch(msgId) {
 ...
33  	case CID_PING_RESP: {
34      	messages::PingResp response;    	//struct that contains response fields
35      	response = data;                	//operator overload fills in response struct with correct data
36      	std::cout << response << std::endl; //operator overload prints out response data
37           	 
38      	//sends another ping to the other beacon, creating a feedback loop between ping sends and ping responses
39      	this->ping_beacon(response.acoFix.srcId, MSG_REQU);
40
41  	} break;
```

Explanation:

* Line 28: switch statement on msgId ensures you decode the right type of message and lets you define event
	handlers for different types of beacon responses.
* Line 33: CID_PING_RESP is the CID_E associated with Ping Response
* Line 35: The assignment is an operator overload that fills in the struct with all the information in data
* Line 36: Another operator overload that prints out all of the response fields
* Line 39: members of the response struct can be used in your c++ code. In this case we're using the beacon
	ID of the beacon that sent the response to send another ping to that beacon.

Besides `SeatracDriver::on_message`, there are two other methods you can use to read a message from the beacon.
`SeatracDriver::send_request` may be used if the message is a response to a command you sent. You can also
use `SeatracDriver::wait_for_message(CID_E msgId, T* data, int64_t timeout)`. `wait_for_message` returns true
if the message was successfully received and false otherwise. It has three arguments:

* `msgId`   : The CID_E code of the message to wait for
* `data`	: A struct that will be filled with the message data from the beacon
* `timeout` : How long to wait for that message to be received

## commands.h

seatrac_driver provides a few high level functions in [commands.h](https://bitbucket.org/frostlab/seatrac_driver/src/main/include/seatrac_driver/commands.h).

A few notable functions in commands.h:

* set_beacon_id: can be used to change the beacons Id
* ping_send: sends a ping to another beacon
* data_send: sends a data message to another beacon

## Seatrac Support Webpage
https://www.blueprintsubsea.com/seatrac/support

This is the link to the latest seatrac beacon user support material.
It has links to the Seatrac Beacon User Guide and the Seatrac Developer User Guide.


