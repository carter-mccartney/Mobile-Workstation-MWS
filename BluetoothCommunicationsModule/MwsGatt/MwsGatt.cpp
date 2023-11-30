// Copyright 2017-2019 Paul Nettle
//
// This file is part of Gobbledegook.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file in the root of the source tree.

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
// >>
// >>>  INSIDE THIS FILE
// >>
//
// This is an example single-file stand-alone application that runs a Gobbledegook server.
//
// >>
// >>>  DISCUSSION
// >>
//
// Very little is required ("MUST") by a stand-alone Sapplication to instantiate a valid Gobbledegook server. There are also some
// things that are reocommended ("SHOULD").
//
// * A stand-alone application MUST:
//
//     * Start the server via a call to `ggkStart()`.
//
//         Once started the server will run on its own thread.
//
//         Two of the parameters to `ggkStart()` are delegates responsible for providing data accessors for the server, a
//         `GGKServerDataGetter` delegate and a 'GGKServerDataSetter' delegate. The getter method simply receives a string name (for
//         example, "battery/level") and returns a void pointer to that data (for example: `(void *)&batteryLevel`). The setter does
//         the same only in reverse.
//
//         While the server is running, you will likely need to update the data being served. This is done by calling
//         `ggkNofifyUpdatedCharacteristic()` or `ggkNofifyUpdatedDescriptor()` with the full path to the characteristic or delegate
//         whose data has been updated. This will trigger your server's `onUpdatedValue()` method, which can perform whatever
//         actions are needed such as sending out a change notification (or in BlueZ parlance, a "PropertiesChanged" signal.)
//
// * A stand-alone application SHOULD:
//
//     * Shutdown the server before termination
//
//         Triggering the server to begin shutting down is done via a call to `ggkTriggerShutdown()`. This is a non-blocking method
//         that begins the asynchronous shutdown process.
//
//         Before your application terminates, it should wait for the server to be completely stopped. This is done via a call to
//         `ggkWait()`. If the server has not yet reached the `EStopped` state when `ggkWait()` is called, it will block until the
//         server has done so.
//
//         To avoid the blocking behavior of `ggkWait()`, ensure that the server has stopped before calling it. This can be done
//         by ensuring `ggkGetServerRunState() == EStopped`. Even if the server has stopped, it is recommended to call `ggkWait()`
//         to ensure the server has cleaned up all threads and other internals.
//
//         If you want to keep things simple, there is a method `ggkShutdownAndWait()` which will trigger the shutdown and then
//         block until the server has stopped.
//
//     * Implement signal handling to provide a clean shut-down
//
//         This is done by calling `ggkTriggerShutdown()` from any signal received that can terminate your application. For an
//         example of this, search for all occurrences of the string "signalHandler" in the code below.
//
//     * Register a custom logging mechanism with the server
//
//         This is done by calling each of the log registeration methods:
//
//             `ggkLogRegisterDebug()`
//             `ggkLogRegisterInfo()`
//             `ggkLogRegisterStatus()`
//             `ggkLogRegisterWarn()`
//             `ggkLogRegisterError()`
//             `ggkLogRegisterFatal()`
//             `ggkLogRegisterAlways()`
//             `ggkLogRegisterTrace()`
//
//         Each registration method manages a different log level. For a full description of these levels, see the header comment
//         in Logger.cpp.
//
//         The code below includes a simple logging mechanism that logs to stdout and filters logs based on a few command-line
//         options to specify the level of verbosity.
//
// >>
// >>>  Building with GOBBLEDEGOOK
// >>
//
// The Gobbledegook distribution includes this file as part of the Gobbledegook files with everything compiling to a single, stand-
// alone binary. It is built this way because Gobbledegook is not intended to be a generic library. You will need to make your
// custom modifications to it. Don't worry, a lot of work went into Gobbledegook to make it almost trivial to customize
// (see Server.cpp).
//
// If it is important to you or your build process that Gobbledegook exist as a library, you are welcome to do so. Just configure
// your build process to build the Gobbledegook files (minus this file) as a library and link against that instead. All that is
// required by applications linking to a Gobbledegook library is to include `include/Gobbledegook.h`.
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include <signal.h>
#include <iostream>
#include <thread>
#include <sstream>
#include <chrono>
#include <vector>

#include <Gobbledegook.h>

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64.hpp>
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/msg/u_int8.hpp>
#include <example_interfaces/msg/int8.hpp>

//
// Constants
//

// Maximum time to wait for any single async process to timeout during initialization
static const int kMaxAsyncInitTimeoutMS = 30 * 1000;

// The class of the node.
class GattNode : public rclcpp::Node
{
public:
	GattNode() : Node("gatt_node")
	{
		this->currentMessage = new std::string();
		using std::placeholders::_1;
		this->userMessageSubscriber = this->create_subscription<example_interfaces::msg::String>("user_message", 10, std::bind(&GattNode::onMessageDelivered, this, _1));
		this->batteryLevelSubscriber = this->create_subscription<example_interfaces::msg::UInt8>("battery_level", 10, std::bind(&GattNode::onBatteryLevelUpdated, this, _1));
		this->isRunningSubscriber = this->create_subscription<example_interfaces::msg::Bool>("follower_mode_is_running_request", 10, std::bind(&GattNode::onIsActivatedChanged, this, _1));
		this->rangeSubscriber = this->create_subscription<example_interfaces::msg::Float64>("follower_range_request", 10, std::bind(&GattNode::onRangeChanged, this, _1));
      	this->rangePublisher = this->create_publisher<example_interfaces::msg::Float64>("range_update", 10);
		this->signaturePublisher = this->create_publisher<example_interfaces::msg::String>("followee_signature", 10);
		this->isActivatedPublisher = this->create_publisher<example_interfaces::msg::Bool>("follower_mode_is_running", 10);
		this->onePublisher = this->create_publisher<example_interfaces::msg::Int8>("calibration_value_one", 10);
		this->twoPublisher = this->create_publisher<example_interfaces::msg::Int8>("calibration_value_two", 10);
		this->threePublisher = this->create_publisher<example_interfaces::msg::Int8>("calibration_value_three", 10);
		this->fourPublisher = this->create_publisher<example_interfaces::msg::Int8>("calibration_value_four", 10);
		this->newOneSubscriber = this->create_subscription<example_interfaces::msg::Int8>("new_calibration_value_one", 10, std::bind(&GattNode::onOneChanged, this, _1));
		this->newTwoSubscriber = this->create_subscription<example_interfaces::msg::Int8>("new_calibration_value_two", 10, std::bind(&GattNode::onTwoChanged, this, _1));
		this->newThreeSubscriber = this->create_subscription<example_interfaces::msg::Int8>("new_calibration_value_three", 10, std::bind(&GattNode::onThreeChanged, this, _1));
		this->newFourSubscriber = this->create_subscription<example_interfaces::msg::Int8>("new_calibration_value_four", 10, std::bind(&GattNode::onFourChanged, this, _1));
		this->isCalibratingPublisher = this->create_publisher<example_interfaces::msg::Bool>("is_calibrating", 10);
		this->isCalibratingSubscriber = this->create_subscription<example_interfaces::msg::Bool>("is_calibrating_return", 10, std::bind(&GattNode::onIsCalibratingUpdated, this, _1));
		this->targetPublisher = this->create_publisher<example_interfaces::msg::UInt8>("calibration_target", 10);
    }
	~GattNode()
	{
		delete this->currentMessage;
	}

	// Publishes the signature to the ROS network.
	void setSignature(std::string signature)
	{
		example_interfaces::msg::String message = example_interfaces::msg::String();
		message.data = signature;
		signaturePublisher.get()->publish(message);
	}

	// Gets whether the follower mode is activated.
	bool getIsActivated()
	{
		return this->isActivated;
	}

	// Sets whether the follower mode is activated.
	void setIsActivated(bool isActivated)
	{
		this->isActivated = isActivated;
		example_interfaces::msg::Bool message = example_interfaces::msg::Bool();
		message.data = isActivated;
		this->isActivatedPublisher.get()->publish(message);
	}

	// Updates whether follower mode is activated based on setting in the ROS network.
	void onIsActivatedChanged(const example_interfaces::msg::Bool& isActivated)
	{
		this->isActivated = isActivated.data;
		ggkNofifyUpdatedCharacteristic("/com/gobbledegook/follower/is_activated");
	}

	// Gets the current range set.
	double getRange()
	{
		return this->range;
	}

	// Sets a new range and publishes.
	void setRange(double range)
	{
		this->range = range;
		example_interfaces::msg::Float64 message = example_interfaces::msg::Float64();
		message.data = range;
		this->rangePublisher.get()->publish(message);
	}

	// Updates the range set based on setting in the ROS network.
	void onRangeChanged(const example_interfaces::msg::Float64& range)
	{
		this->range = range.data;
		ggkNofifyUpdatedCharacteristic("/com/gobbledegook/follower/range");
	}

	// Gets the current battery level.
	uint8_t getBatteryLevel()
	{
		return this->batteryLevel;
	}

	// Modifies the battery level on update.
	void onBatteryLevelUpdated(const example_interfaces::msg::UInt8& batteryLevel)
	{
		this->batteryLevel = batteryLevel.data;
		ggkNofifyUpdatedCharacteristic("/com/gobbledegook/battery/level");
	}

	// Gets the most recent message delivered.
	std::string* getMessage() const
	{
		return this->currentMessage;
	}
	
	// Updates the message on delivery and sends it.
	void onMessageDelivered(const example_interfaces::msg::String& message)
	{
		this->currentMessage->clear();
		for(std::size_t i = 0; i < message.data.size(); i++)
		{
			this->currentMessage->push_back(message.data[i]);
		}
		ggkNofifyUpdatedCharacteristic("/com/gobbledegook/messenger/message");
	}

	// Gets the currently stored calibration for one.
	int8_t getOne()
	{
		return this->oneCalibrationValue;
	}

	// Sets the currently stored calibration for one.
	void setOne(int8_t one)
	{
		this->oneCalibrationValue = one;
		example_interfaces::msg::Int8 message;
		message.data = one;
		this->onePublisher->publish(message);
	}

	// Updates the value of one.
	void onOneChanged(const example_interfaces::msg::Int8& message)
	{
		this->oneCalibrationValue = message.data;
	}

	// Gets the currently stored calibration for two.
	int8_t getTwo()
	{
		return this->twoCalibrationValue;
	}

	// Sets the currently stored calibration for two.
	void setTwo(int8_t two)
	{
		this->twoCalibrationValue = two;
		example_interfaces::msg::Int8 message;
		message.data = two;
		this->twoPublisher->publish(message);
	}

	// Updates the value of two.
	void onTwoChanged(const example_interfaces::msg::Int8& message)
	{
		this->twoCalibrationValue = message.data;
	}

	// Gets the currently stored calibration for three.
	int8_t getThree()
	{
		return this->threeCalibrationValue;
	}

	// Sets the currently stored calibration for three.
	void setThree(int8_t three)
	{
		this->threeCalibrationValue = three;
		example_interfaces::msg::Int8 message;
		message.data = three;
		this->threePublisher->publish(message);
	}

	// Updates the value of three.
	void onThreeChanged(const example_interfaces::msg::Int8& message)
	{
		this->threeCalibrationValue = message.data;
	}

	// Gets the currently stored calibration for four.
	int8_t getFour()
	{
		return this->fourCalibrationValue;
	}

	// Sets the currently stored calibration for four.
	void setFour(int8_t four)
	{
		this->fourCalibrationValue = four;
		example_interfaces::msg::Int8 message;
		message.data = four;
		this->fourPublisher->publish(message);
	}

	// Updates the value of four.
	void onFourChanged(const example_interfaces::msg::Int8& message)
	{
		this->fourCalibrationValue = message.data;
	}

	// Sets whether calibration has started.
	void setIsCalibrating(bool value)
	{
		example_interfaces::msg::Bool message = example_interfaces::msg::Bool();
		message.data = value;
		this->isCalibratingPublisher.get()->publish(message);
	}

	// Notifies that calibration has comlpeted.
	void onIsCalibratingUpdated(const example_interfaces::msg::Bool& message)
	{
		ggkNofifyUpdatedCharacteristic("/com/gobbledegook/calibration/is_calibrating");
	}

	// Sets the current target.
	void setTarget(uint8_t target)
	{
		example_interfaces::msg::UInt8 message;
		message.data = target;
		this->targetPublisher->publish(message);
	}

private:
	// Whether follower mode is activated.
	bool isActivated = false;

	// The current follower range.
	double range = 5;

	// The current battery level.
	uint8_t batteryLevel = 100;

	// The most recent message delivered.
	std::string* currentMessage;

	// The calibration value of one.
	uint8_t oneCalibrationValue = -69;

	// The calibration value of two.
	uint8_t twoCalibrationValue = -69;

	// The calibration value of three.
	uint8_t threeCalibrationValue = -69;

	// The calibration value of four.
	uint8_t fourCalibrationValue = -69;

	// The publisher for the current range.
	rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr rangePublisher;

	// The publisher for the current running state.
	rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr isActivatedPublisher;

	// The publisher for the current signature.
	rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr signaturePublisher;

	// The subscriber for the message.
	rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr userMessageSubscriber;

	// The subscriber for the battery level.
	rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr batteryLevelSubscriber;

	// The subscriber for is running.
	rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr isRunningSubscriber;

	// The subscriber for the range.
	rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr rangeSubscriber;

	// The publisher for the calibration of one.
	rclcpp::Publisher<example_interfaces::msg::Int8>::SharedPtr onePublisher;

	// The subscriber for the calibration of one.
	rclcpp::Subscription<example_interfaces::msg::Int8>::SharedPtr newOneSubscriber;

	// The publisher for the calibration of two.
	rclcpp::Publisher<example_interfaces::msg::Int8>::SharedPtr twoPublisher;

	// The subscriber for the calibration of two.
	rclcpp::Subscription<example_interfaces::msg::Int8>::SharedPtr newTwoSubscriber;

	// The publisher for the calibration of three.
	rclcpp::Publisher<example_interfaces::msg::Int8>::SharedPtr threePublisher;

	// The subscriber for the calibration of three.
	rclcpp::Subscription<example_interfaces::msg::Int8>::SharedPtr newThreeSubscriber;

	// The publisher for the calibration of four.
	rclcpp::Publisher<example_interfaces::msg::Int8>::SharedPtr fourPublisher;

	// The subscriber for the calibration of four.
	rclcpp::Subscription<example_interfaces::msg::Int8>::SharedPtr newFourSubscriber;

	// The publisher for whether calibration is on-going.
	rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr isCalibratingPublisher;

	// The subscriber for the whether calibration is on-going.
	rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr isCalibratingSubscriber;

	// The publisher for the calibration target.
	rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr targetPublisher;
};

// The ROS node for communications.
static std::shared_ptr<GattNode> node;

// The number of cycles since the last acknowledge.
static uint32_t cyclesSinceAcknowledge;

// Whether an acknowledge is being awaited.
static bool isAwaitingAcknowledge;

//
// Logging
//

enum LogLevel
{
	Debug,
	Verbose,
	Normal,
	ErrorsOnly
};

// Our log level - defaulted to 'Normal' but can be modified via command-line options
LogLevel logLevel = Normal;

// Our full set of logging methods (we just log to stdout)
//
// NOTE: Some methods will only log if the appropriate `logLevel` is set
void LogDebug(const char *pText) { if (logLevel <= Debug) { std::cout << "  DEBUG: " << pText << std::endl; } }
void LogInfo(const char *pText) { if (logLevel <= Verbose) { std::cout << "   INFO: " << pText << std::endl; } }
void LogStatus(const char *pText) { if (logLevel <= Normal) { std::cout << " STATUS: " << pText << std::endl; } }
void LogWarn(const char *pText) { std::cout << "WARNING: " << pText << std::endl; }
void LogError(const char *pText) { std::cout << "!!ERROR: " << pText << std::endl; }
void LogFatal(const char *pText) { std::cout << "**FATAL: " << pText << std::endl; }
void LogAlways(const char *pText) { std::cout << "..Log..: " << pText << std::endl; }
void LogTrace(const char *pText) { std::cout << "-Trace-: " << pText << std::endl; }

//
// Signal handling
//

// We setup a couple Unix signals to perform graceful shutdown in the case of SIGTERM or get an SIGING (CTRL-C)
void signalHandler(int signum)
{
	switch (signum)
	{
		case SIGINT:
			LogStatus("SIGINT recieved, shutting down");
			ggkTriggerShutdown();
			break;
		case SIGTERM:
			LogStatus("SIGTERM recieved, shutting down");
			ggkTriggerShutdown();
			break;
	}
}

//
// Server data management
//

// The value returned for the range, which must be stored as a global variable because of a strange issue with getting the address from the shared pointer.
double rangeResult;
bool isActivatedResult;
uint8_t batteryLevelResult;
int8_t oneResult;
int8_t twoResult;
int8_t threeResult;
int8_t fourResult;

// Called by the server when it wants to retrieve a named value
//
// This method conforms to `GGKServerDataGetter` and is passed to the server via our call to `ggkStart()`.
//
// The server calls this method from its own thread, so we must ensure our implementation is thread-safe. In our case, we're simply
// sending over stored values, so we don't need to take any additional steps to ensure thread-safety.
const void *dataGetter(const char *pName)
{
	if (nullptr == pName)
	{
		LogError("NULL name sent to server data getter");
		return nullptr;
	}
	else
	{
		std::string serviceCharacteristic = pName;

		// Put the actions in response to a client read here.
		if(serviceCharacteristic == "battery/level")
		{
			batteryLevelResult = node.get()->getBatteryLevel();
			return &batteryLevelResult;
		}
		else if(serviceCharacteristic == "follower/is_activated")
		{
			isActivatedResult = node.get()->getIsActivated();
			return &isActivatedResult;
		}
		else if(serviceCharacteristic == "follower/range")
		{
			rangeResult = node.get()->getRange();
			return &rangeResult;
		}
		else if(serviceCharacteristic == "messenger/message")
		{
			return node.get()->getMessage()->c_str();
		}
		else if(serviceCharacteristic == "calibration/one")
		{
			oneResult = node.get()->getOne();
			return &oneResult;
		}
		else if(serviceCharacteristic == "calibration/two")
		{
			twoResult = node.get()->getTwo();
			return &twoResult;
		}
		else if(serviceCharacteristic == "calibration/three")
		{
			threeResult = node.get()->getThree();
			return &threeResult;
		}
		else if(serviceCharacteristic == "calibration/four")
		{
			fourResult = node.get()->getFour();
			return &fourResult;
		}

		return nullptr;
	}
}

// Called by the server when it wants to update a named value
//
// This method conforms to `GGKServerDataSetter` and is passed to the server via our call to `ggkStart()`.
//
// The server calls this method from its own thread, so we must ensure our implementation is thread-safe. In our case, we're simply
// sending over stored values, so we don't need to take any additional steps to ensure thread-safety.
int dataSetter(const char *pName, const void *pData)
{
	if (nullptr == pName)
	{
		LogError("NULL name sent to server data setter");
		return 0;
	}
	else if (nullptr == pData)
	{
		LogError("NULL pData sent to server data setter");
		return 0;
	}
	else
	{
		std::string serviceCharacteristic = pName;
		
		// Put the actions on response to a client write here.
		if(serviceCharacteristic == "follower/is_activated")
		{
			bool isActivated = *(bool*)pData;
			node.get()->setIsActivated(isActivated);
			return 1;
		}
		else if(serviceCharacteristic == "follower/signature")
		{
			char* signature = (char*)pData;
			node.get()->setSignature(signature);
			return 1;
		}
        else if(serviceCharacteristic == "follower/range")
        {
            uint8_t range = *(uint8_t*)pData;
            node.get()->setRange(range / 2.0);
            return 1;
        }
		else if(serviceCharacteristic == "follower/acknowledge")
		{
			cyclesSinceAcknowledge = 0;
			isAwaitingAcknowledge = false;

			// TODO: Update ESP-32s with new challenge.
			return 1;
		}
		else if(serviceCharacteristic == "calibration/one")
		{
			int8_t one = *(int8_t*)pData;
			node->setOne(one);
			return 1;
		}
		else if(serviceCharacteristic == "calibration/two")
		{
			int8_t two = *(int8_t*)pData;
			node->setTwo(two);
			return 1;
		}
		else if(serviceCharacteristic == "calibration/three")
		{
			int8_t three = *(int8_t*)pData;
			node->setThree(three);
			return 1;
		}
		else if(serviceCharacteristic == "calibration/four")
		{
			int8_t four = *(int8_t*)pData;
			node->setFour(four);
			return 1;
		}
		else if(serviceCharacteristic == "calibration/is_calibrating")
		{
			bool isCalibrating = *(bool*)pData;
			node->setIsCalibrating(isCalibrating);
			return 1;
		}
		else if(serviceCharacteristic == "calibration/target")
		{
			uint8_t target = *(uint8_t*)pData;
			node->setTarget(target);
			return 1;
		}

		return 0;
	}
}

void reset();

/*
 * The starting point for all ROS code.
 * 
 */
void gattMain()
{
	// Setup our signal handlers
	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);

	// Register our loggers
	ggkLogRegisterDebug(LogDebug);
	ggkLogRegisterInfo(LogInfo);
	ggkLogRegisterStatus(LogStatus);
	ggkLogRegisterWarn(LogWarn);
	ggkLogRegisterError(LogError);
	ggkLogRegisterFatal(LogFatal);
	ggkLogRegisterAlways(LogAlways);
	ggkLogRegisterTrace(LogTrace);

	// Start the server's ascync processing
	//
	// This starts the server on a thread and begins the initialization process
	//
	// !!!IMPORTANT!!!
	//
	//     This first parameter (the service name) must match tha name configured in the D-Bus permissions. See the Readme.md file
	//     for more information.
	//
	if (!ggkStart("gobbledegook", dataGetter, dataSetter, kMaxAsyncInitTimeoutMS))
	{
		exit(1);
	}

	// Initialize server variables.
	reset();

	// Wait for the server to start the shutdown process
	uint8_t currentIteration = 0;
	while (ggkGetServerRunState() < EStopping)
	{
		std::this_thread::sleep_for(std::chrono::seconds(15));

		// Update GATT server state.
		if(getActiveConnections() == 0)
		{
			// No one is connected.
			reset();
			currentIteration = 0;
		}
		else
		{
			// Put periodic updates here.
			if(node.get()->getIsActivated())
			{
				// Check whether the state is awaiting acknowledge or normal operation.
				if(isAwaitingAcknowledge)
				{
					cyclesSinceAcknowledge++;
					if(cyclesSinceAcknowledge > 20)
					{
						cyclesSinceAcknowledge = 0;
						isAwaitingAcknowledge = false;
						ggkNofifyUpdatedCharacteristic("/com/gobbledegook/follower/change_advertisement");
					}
				}
				else
				{
					// Check whether to request a new advertisement.
					currentIteration++;
					if(currentIteration % 1 == 0)
					{
						// Change the current advertisement.
						ggkNofifyUpdatedCharacteristic("/com/gobbledegook/follower/change_advertisement");
					}
				}
			}
		}
	}

	// Wait for the server to come to a complete stop (CTRL-C from the command line)
	if (!ggkWait())
	{
		exit(1);
	}
}

//
// Entry point
//
int main(int argc, char **argv)
{
	// A basic command-line parser
	for (int i = 1; i < argc; ++i)
	{
		std::string arg = argv[i];
		if (arg == "-q")
		{
			logLevel = ErrorsOnly;
		}
		else if (arg == "-v")
		{
			logLevel = Verbose;
		}
		else if  (arg == "-d")
		{
			logLevel = Debug;
		}
		else
		{
			LogFatal((std::string("Unknown parameter: '") + arg + "'").c_str());
			LogFatal("");
			LogFatal("Usage: MwsGatt [-q | -v | -d]");
			return -1;
		}
	}

	// Start GATT server.
  	rclcpp::init(0, nullptr);
	node = std::make_shared<GattNode>();
	std::thread gattThread(gattMain);

  	rclcpp::spin(node);

	// Wait for gatt server to be done.
	gattThread.join();
  	rclcpp::shutdown();

	// Return the final server health status as a success (0) or error (-1)
  	return ggkGetServerHealth() == EOk ? 0 : 1;
}

void reset()
{
	node.get()->setIsActivated(false);
	node.get()->setRange(1.0);
	cyclesSinceAcknowledge = 0;
	isAwaitingAcknowledge = false;
}