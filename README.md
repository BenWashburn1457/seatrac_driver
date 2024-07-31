# FRoSt seatrac_driver

This is a c++ driver for the Blueprint Subsea Seatrac USBL beacon.

It is a significantly modified fork of the seatrac_driver written by Pierre Narvor:
https://gitlab.ensta-bretagne.fr/narvorpi/seatrac_driver.

If this is your first time using the driver, there are four c++ examples you can try 
to test out the beacons acoustic transmission capabilities.

This repository also includes a ros2 node built on top of the c++ driver. If you don't
plan on using the driver at the c++ level, you can skip to the ROS2 section of
the readme.

## Installation

This is a standard cmake package. It can be installed manually and found by
your project using `CMAKE_PREFIX_PATH`, or installed automatically in your cmake
file using `FetchContent`.

### Automatic Installation using CMake
Using FetchContent in your CMake, you can automatically download and install
seatrac_driver from its git repository. Simply include
```
include(FetchContent)
FetchContent_Declare(seatrac_driver
	GIT_REPOSITORY https://Clayton314@bitbucket.org/frostlab/seatrac_driver.git
	GIT_TAG        main
)
FetchContent_MakeAvailable(seatrac_driver)
```
in your CMake file and you're good to go.

### Manual Installation
While using FetchContent is the simplest and least likely to encounter errors,
you also have the option to install the driver manually.
To install manually, run the following set of bash commands:

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

Then, add `find_package(seatrac_driver REQUIRED)` to your CMake.

### CMake
This is an example of what the CMake file for your project might look like.
This CMake first tries to find a local installation. If that doesn't work,
it downloads seatrac_driver from the git repository.
```
cmake_minimum_required(VERSION 3.6)
project(your_project VERSION 0.1)

find_package(seatrac_driver QUIET)
if(NOT TARGET seatrac_driver)
    include(FetchContent)
    FetchContent_Declare(seatrac_driver
        GIT_REPOSITORY https://Clayton314@bitbucket.org/frostlab/seatrac_driver.git
        GIT_TAG        main
    )
    FetchContent_MakeAvailable(seatrac_driver)
endif()

add_executable(your_execuatable ...)
target_link_libraries(your_executable PRIVATE seatrac_driver)
```

### Installation Troubleshooting
Some common issues that may occur during installation

* Cannot find Boost: Boost is another dependency of seatrac_driver. You can install it with
  ```sudo apt-get install libboost-all-dev```
* Cannot find librtac_asio.so: rtac_asio is a dependency of seatrac_driver. 
	The CMake file for seatrac_driver first looks for a local installation, 
	and then pulls it from the git repository if it can't find it locally.
	You can install it from https://github.com/pnarvor/rtac_asio.
	

## Examples

This driver includes c++ examples for each of the 4 acoustic message protocols:
PING, DAT, ECHO, and NAV. For new users, these examples are a good place to start.

### To run each example:

1. Connect 2 beacons to your computer and place them together in water.
2. Navigate to the example's folder `seatrac_driver/examples/<example_name>`
3. Build the example:
	```
	mkdir build && cd build
	cmake ..
	make
	cd ..
	```
	Each example is setup to find and download the driver from this git
	repository (using FetchContent) if it cannot find an existing
	seatrac_driver on your computer.
4. Run the example: `sudo ./build/<example_name> <serial_port>`
	The executable name is the same as the example folder name.
	It takes one argument - the serial port that the seatrac modem is
	connected too (for example `/dev/ttyUSB0`).
	


## Using with c++
	
You can interface with the beacon by subclassing `SeatracDriver` class from `SeatracDriver.h`,
and initializing it with the serial port connection as an argument:

```
#include <seatrac_driver/Seatrac_Driver.h>
using namespace narval::seatrac;

class MyDriver : public SeatracDriver {
}

int main() {
	std::string serial_port = "/dev/ttyUSB0";
	MyDriver seatrac(serial_port); //initialize your driver
	getchar(); //wait for user input before terminating the program
	return 0;
}
```

### Reading messages from the beacon (c++)

All messages received from the beacon will result in a call to the SeatracDriver::on_message 
method. The first argument of on_message is the message id, a uint8 that tells you what
type of message you received. msgId is defined by the CID_E enum from SeatracEnums.h.
You can access the contents of the message by copying its data into a message struct. 
Message structs can be found in the include/seatrac_driver/messages folder. You can only
fill a message struct of the same type as the msgId. Ideally, this can be ensured with
a switch statement.

```
class MyDriver : public SeatracDriver {
	void on_message(CID_E msgId, const std::vector<uint8_t>& data) {
		if(msgId == CID_PING_RESP) { 				//assigning a PingResp to data when msgId != CID_PING_RESP will throw an error
				messages::PingResp response;        //struct that contains response fields
				response = data;                    //operator overload fills in response struct with correct data
				std::cout << response << std::endl; //operator overload prints out response data
		}
	}
}
```

### Sending messages too the beacon (c++)

You can send a message to the beacon using the SeatracDriver::send method.
SeatracDriver::send takes a byte array pointer and number of bytes as arguments.
First, create a class of type <MessageType>::Request. Specify the request struct
fields, then send it by recasting the struct as a byte array pointer. When sending
a message, you do not have to specify the CID_E message id, since the Request 
struct has already defined it for you.

```
class MyDriver : public SeatracDriver {
	void ping_beacon_15() {
		messages::PingSend::Request request;	//the message id for PingSend is CID_PING_SEND
		request.target = BEACON_ID_15;			//BID_E enum type. beacon id.
		request.pingType = MSG_REQU;			//AMSGTYPE_E enum type. requests a usbl response from beacon 15
		this->send(sizeof(request), (const uint8_t*)&request);
	}
}
```

### Other useful features

* commands.h - Provides a set of higher level functions to quickly write code
	that works with the beacon. It has functions that change settings, change
	the beacon id, or send acoustic messages.
* SeatracDriver::send_request and SeatracDriver::wait_for_message - send_request sends a 
	request to the beacon and blocks until the beacon returns a response message. It
	dumps the returned data in a response structure pointer argument. wait_for_data
	blocks until the beacon returns a message with the correct CID_E. It then fills
	the response structure with the data received.

	```
	SeatracDriver seatrac(serial_port);
	messages::PingSend response;
	messages::PingSend::Request request;
	request.target = BEACON_ID_15;
	request.pingType = MSG_REQU;
	seatrac.send_request(sizeof(request), (const uint8_t*)&request, &response);
	messages::PingResp ping_resp;
	seatrac.wait_for_data(CID_PING_RESP, &ping_resp);
	```



## Using with ROS2

For applications using ros, the ros2 seatrac node provides a high level interface 
with the beacon. It includes two interfaces: ModemSend and ModemRec. 
ModemSend is used to instruct the beacon to send acoustic messages.
ModemRec is used to get information from the beacon. 

The ros2 node is designed mainly to send and interpret acoustic transmissions. It does 
not support changing settings, setting beacon id, calibration, or diagnostics. These 
functions can be achieved using the c++ interface. A seatrac_setup_tool has also been
provided with a command line interface that lets you quickly setup any beacon.

### To run ROS2:

1. Navigate to `seatrac_driver/tools/ros2_seatrac`
2. Build the example and source local setup: `colcon build && source ./install/setup.bash`
3. Run: `ros2 run seatrac modem`

### Reading messages from the beacon (ros2)

In a separate node, simply subscribe to the ModemRec interface. ModemRec has many
fields, most of which are only used for certain message types. The field msg_id, a uint8 
representing the message CID_E, indicates what type of message was sent and hence what 
other fields are populated by that message. While the seatrac ros node publishes a 
ModemRec for every message received from the beacon, it may not include all the 
information the original message contained. However, it does include all the fields
necessary for acoustic transmissions, USBL data, and error messages. 

Review [ModemRec.msg here](https://bitbucket.org/frostlab/seatrac_driver/src/main/tools/ros2_seatrac/seatrac_interfaces/msg/ModemRec.msg)

### Sending acoustic transmission commands (ros2)

ModemSend allows you to send commands to the beacon, but only commands that instruct 
the beacon to transmit an acoustic message. The field msg_id is the CID_E of the message
and can only take 4 values: CID_PING_SEND, CID_DAT_SEND, CID_ECHO_SEND, or CID_NAV_QUERY_SEND.

Review [ModemSend.msg here](https://bitbucket.org/frostlab/seatrac_driver/src/main/tools/ros2_seatrac/seatrac_interfaces/msg/ModemSend.msg)

## Seatrac Messages

TODO
























































































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


