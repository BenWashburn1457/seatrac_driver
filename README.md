# Seatrac x150/x110 Driver

This is a c++ driver for the Blueprint Subsea Seatrac x100 series of USBL beacons.

It is a fork of the seatrac_driver written by Pierre Narvor:
https://gitlab.ensta-bretagne.fr/narvorpi/seatrac_driver.

If this is your first time using the driver, there are four c++ examples you can try
to test out the beacon's acoustic transmission capabilities.

This repository also includes a ros2 node built on top of the c++ driver. If you don't
plan on using the driver at the c++ level, you can skip to the ROS2 section of
the readme.

## Installation

This is a standard cmake package. It can be installed manually and found by
your project using `CMAKE_PREFIX_PATH`, or installed automatically in your cmake
file using `FetchContent`.

#### Automatic Installation using CMake
Using FetchContent in your CMake, you can automatically download and install
seatrac_driver from its git repository. Simply include

```cmake
include(FetchContent)
FetchContent_Declare(seatrac_driver
	GIT_REPOSITORY https://Clayton314@bitbucket.org/frostlab/seatrac_driver.git
	GIT_TAG    	main
)
FetchContent_MakeAvailable(seatrac_driver)
```

in your CMake file and you're good to go.

#### Manual Installation
While using FetchContent is the simplest and least likely to encounter errors,
you also have the option to install the driver manually.
To install manually, run the following set of bash commands:

```bash
git clone https://Clayton314@bitbucket.org/frostlab/seatrac_driver.git

mkdir build && cd build

cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=<your_install_location> ..

make -j4 install
```

Make sure <your_install_location> is in the CMAKE_PREFIX_PATH environment
variable.

```bash
echo $CMAKE_PREFIX_PATH
```

If not, be sure to add it (for example with the following line :)

```bash
echo "export CMAKE_PREFIX_PATH=<your_install_location_full_path>:\$CMAKE_PREFIX_PATH" >> .bashrc
```

Then, add `find_package(seatrac_driver REQUIRED)` to your CMake.

#### CMake
This is an example of what the CMake file for your project might look like.
This CMake first tries to find a local installation. If that doesn't work,
it downloads seatrac_driver from the git repository.

```cmake
cmake_minimum_required(VERSION 3.6)
project(your_project VERSION 0.1)

find_package(seatrac_driver QUIET)
if(NOT TARGET seatrac_driver)
	include(FetchContent)
	FetchContent_Declare(seatrac_driver
    	GIT_REPOSITORY https://Clayton314@bitbucket.org/frostlab/seatrac_driver.git
    	GIT_TAG    	main
	)
	FetchContent_MakeAvailable(seatrac_driver)
endif()

add_executable(your_execuatable ...)
target_link_libraries(your_executable PRIVATE seatrac_driver)
```

#### Installation Troubleshooting
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

#### To run each example:

1. Connect 2 beacons to your computer and place them together in water.
2. Navigate to the example's folder `seatrac_driver/examples/<example_name>`
3. Build the example:
	```bash
	mkdir build && cd build
	cmake ..
	make
	cd ..
	```
	Each example is setup to find and download the driver from this git
	repository (using FetchContent) if it cannot find an existing
	seatrac_driver on your computer.
4. Run the example: `./build/<example_name> <serial_port>`
	The executable name is the same as the example folder name.
	It takes one argument - the serial port that the seatrac modem is
	connected too (for example `/dev/ttyUSB0`).
    


## Using with c++
    
You can interface with the beacon by subclassing `SeatracDriver` class from `SeatracDriver.h`,
and initializing it with the serial port connection as an argument:

```c++
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

#### Reading messages from the beacon (c++)

All messages received from the beacon will result in a call to the `SeatracDriver::on_message`
method. The first argument of on_message is the message id, a uint8 that tells you what
type of message you received. msgId is defined by the CID_E enum from SeatracEnums.h.
You can access the contents of the message by copying its data into a message struct.
Message structs can be found in the include/seatrac_driver/messages folder. You can only
fill a message struct of the same type as the msgId. Ideally, this can be ensured with
a switch statement.

```c++
class MyDriver : public SeatracDriver {
	void on_message(CID_E msgId, const std::vector<uint8_t>& data) {
    	if(msgId == CID_PING_RESP) {             	//assigning a PingResp to data when msgId != CID_PING_RESP will throw an error
            	messages::PingResp response;    	//struct that contains response fields
            	response = data;                	//operator overload fills in response struct with correct data
            	std::cout << response << std::endl; //operator overload prints out response data
    	}
	}
}
```

#### Sending messages too the beacon (c++)

You can send a message to the beacon using the `SeatracDriver::send` method.
`SeatracDriver::send` takes a byte array pointer and number of bytes as arguments.
First, create a class of type `<MessageType>::Request`. Specify the request struct
fields, then send it by recasting the struct as a byte array pointer. When sending
a message, you do not have to specify the CID_E message id, since the Request
struct has already defined it for you.

```c++
class MyDriver : public SeatracDriver {
	void ping_beacon_15() {
    	messages::PingSend::Request request;	//the message id for PingSend is CID_PING_SEND
    	request.target = BEACON_ID_15;        	//BID_E enum type. beacon id.
    	request.pingType = MSG_REQU;        	//AMSGTYPE_E enum type. requests a usbl response from beacon 15
    	this->send(sizeof(request), (const uint8_t*)&request);
	}
}
```

#### Other features

* commands.h - Provides a set of higher level functions to quickly write code
	that works with the beacon. It has functions that change settings, change
	the beacon id, or send acoustic messages.
* `SeatracDriver::send_request` and `SeatracDriver::wait_for_message` - `send_request` sends a
	request to the beacon and blocks until the beacon returns a response message. It
	dumps the returned data in a response structure pointer argument. `wait_for_data`
	blocks until the beacon returns a message with the correct CID_E. It then fills
	the response structure with the data received.

	```c++
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
with the beacon. 

the ros2_seatrac_ws workspace contains 3 packages:
* seatrac - a package containing a node to communicate with the seatrac modem
* py_pinger - a package with a node that sends pings using the seatrac node
* seatrac_interfaces - A package with two interfaces: ModemSend and ModemRec.
	* ModemSend is used to instruct the beacon to send acoustic messages.
	* ModemRec is used to get information from the beacon.

The ros2 seatrac node is designed mainly to send and interpret acoustic transmissions. 
It does not support changing settings, setting beacon id, calibration, or diagnostics. 
These functions can be achieved using the c++ interface.

#### To run ROS2:

1. Source your local ros2 distibution `source /opt/ros/<distro>/setup.bash`.
	The ros disto this workspace was developed on is Humble.
1. Navigate to `seatrac_driver/tools/ros2_seatrac_ws`
2. Build the workspace and source local setup: `colcon build && source ./install/setup.bash`
3. Launch: `ros2 launch launch.py`. This launches the seatrac and python pinger nodes,
	and executes 'ros2 topic echo /modem_rec'. To run the seatrac node alone, execute
	`ros2 run seatrac modem`.

#### Reading messages from the beacon (ros2)

Run seatrac modem and subscribe to the ModemRec interface. ModemRec has many
fields, most of which are only used for certain message types. The field msg_id, a uint8
representing the message CID_E, indicates what type of message was sent and hence what
other fields are populated by that message. While the seatrac ros node publishes a
ModemRec for every message received from the beacon, it may not include all the
information the original message contained. However, it does include all the fields
necessary for acoustic transmissions, USBL data, and error messages.

Link to ModemRec: [ModemRec.msg](https://bitbucket.org/frostlab/seatrac_driver/src/main/tools/ros2_seatrac/seatrac_interfaces/msg/ModemRec.msg)

#### Sending acoustic transmission commands (ros2)

ModemSend allows you to send commands to the beacon, but only commands that instruct
the beacon to transmit an acoustic message. The field msg_id is the CID_E of the message
and can only take 4 values: CID_PING_SEND, CID_DAT_SEND, CID_ECHO_SEND, or CID_NAV_QUERY_SEND.

Link to ModemSend: [ModemSend.msg](https://bitbucket.org/frostlab/seatrac_driver/src/main/tools/ros2_seatrac/seatrac_interfaces/msg/ModemSend.msg)



## Seatrac Setup Tool

The Seatrac Setup Tool, located at seatrac_driver/tools/seatrac_setup_tool, is a terminal
program that helps set the beacon id and remaining settings and calibrate a beacon
before use. compile and run it the same way you would any of the examples. See
[Seatrac User Guide pg 18](https://www.blueprintsubsea.com/downloads/seatrac/UM-140-P00918-04.pdf#page=18)
for calibration instructions.


## Seatrac Message Formats

seatrac_driver works by sending messages and receiving messages through a serial line with the beacon.
Every message has a message id, defined in the CID_E enum. the message id tells you what information
that message includes.

The format for seatrac messages is defined in the [Seatrac Developer User Guide](https://www.blueprintsubsea.com/downloads/seatrac/UM-140-D00221-07.pdf).
It defines the [CID_E](https://www.blueprintsubsea.com/downloads/seatrac/UM-140-D00221-07.pdf#page=40)
enum, along with the remaining contents for each message type.

The following is a light overview of some of the seatrac messages.

#### Acoustic Protocol Messages

The seatrac beacon has 4 [acoustic protocols](https://www.blueprintsubsea.com/downloads/seatrac/UM-140-D00221-07.pdf#page=17).
All four protocols are capable of sending acoustic signals with usbl data. They differ in how and what data is
transferred with the message payload.

* PING: Acoustic transmissions without a data payload
* DAT: Acoustic transmissions with a data payload
* ECHO: Acoustic transmissions with a data payload where the remote beacon
	additionally transmits the same payload back to the sender.
* NAV: Acoustic transmissions used to query sensor information from a remote beacon

Each protocol has its own unique set of messages. The following
are messages specifically for PING, but they generalize to the other protocols as well:

* CID_PING_SEND: Instructs the beacon to send a ping to a remote beacon
* CID_PING_REQ: Generated when the beacon receives a request from a remote beacon that it will respond to
* CID_PING_RESP: Generated when the beacon receives a response from a remote beacon
* CID_PING_ERROR: Generated when a ping error occurs.

Enums and Structs used in Acoustic messages:

* ACOMSG_T: This struct contains all the usbl and positioning information from an acoustic transmission.
* AMSGTYPE_E: Indicates whether or not the remote beacon responds and whether or not to include usbl data.
* BID_E: The beacon id between 1 and 15. Used to address acoustic messages.
* NAV_QUERY_T: A bit mask indicating what information to query in a nav message.

#### Beacon Management Messages

Beacon Management messages are used to

* CID_STATUS: This message is output at a regular user defined interval. It contains
	useful information from the beacon, such as the outputs of its auxiliary sensors
* CID_SETTINGS_... : used to manage the beacons settings
* CID_XCVR_... : diagnostic data for the acoustic transceiver independent of acoustic protocols

Enums and Structs used in Beacon Management messages:

* SETTINGS_T: a struct containing all the settings of the beacon
* CST_E: Command status code. Indicates whether or not a command completed successfully and identify errors.

#### Where things are in seatrac_driver

* All seatrac messages are defined in the [messages folder](https://bitbucket.org/frostlab/seatrac_driver/src/main/include/seatrac_driver/messages/)
* All enums (such as CID_E, AMSGTYPE_E, etc) are defined in [SeatracEnums.h](https://bitbucket.org/frostlab/seatrac_driver/src/main/include/seatrac_driver/SeatracEnums.h)
* All non-message structures (such as ACOMSG_T or SETTINGS_T) are defined in [SeatracTypes.h](https://bitbucket.org/frostlab/seatrac_driver/src/main/include/seatrac_driver/SeatracTypes.h)



## Seatrac Support Webpage

https://www.blueprintsubsea.com/seatrac/support
