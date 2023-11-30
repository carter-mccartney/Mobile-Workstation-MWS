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
// This is the money file. This is your server description and complete implementation. If you want to add or remove a Bluetooth
// service, alter its behavior, add or remove characteristics or descriptors (and more), then this is your new home.
//
// >>
// >>>  DISCUSSION
// >>
//
// The use of the term 'server', as it is used here, refers a collection of BlueZ services, characteristics & Descripors (plus
// a little more.)
//
// Our server needs to be described in two ways. Why two? Well, think about it like this: We're communicating with Bluetooth
// clients through BlueZ, and we're communicating with BlueZ through D-Bus. In essence, BlueZ and D-Bus are acting as tunnels, one
// inside the other.
//
// Here are those two descriptions in a bit more detail:
//
// 1. We need to describe ourselves as a citizen on D-Bus: The objects we implement, interfaces we provide, methods we handle, etc.
//
//    To accomplish this, we need to build an XML description (called an 'Introspection' for the curious readers) of our DBus
//    object hierarchy. The code for the XML generation starts in DBusObject.cpp (see `generateIntrospectionXML`) and carries on
//    throughout the other DBus* files (and even a few Gatt* files).
//
// 2. We also need to describe ourselves as a Bluetooth citizen: The services we provide, our characteristics and descriptors.
//
//    To accomplish this, BlueZ requires us to implement a standard D-Bus interface ('org.freedesktop.DBus.ObjectManager'). This
//    interface includes a D-Bus method 'GetManagedObjects', which is just a standardized way for somebody (say... BlueZ) to ask a
//    D-Bus entity (say... this server) to enumerate itself. This is how BlueZ figures out what services we offer. BlueZ will
//    essentially forward this information to Bluetooth clients.
//
// Although these two descriptions work at different levels, the two need to be kept in sync. In addition, we will also need to act
// on the messages we receive from our Bluetooth clients (through BlueZ, through D-Bus.) This means that we'll have yet another
// synchronization issue to resolve, which is to ensure that whatever has been asked of us, makes its way to the correct code in
// our description so we do the right thing.
//
// I don't know about you, but when dealing with data and the concepts "multiple" and "kept in sync" come into play, my spidey
// sense starts to tingle. The best way to ensure sychronization is to remove the need to keep things sychronized.
//
// The large code block below defines a description that includes all the information about our server in a way that can be easily
// used to generate both: (1) the D-Bus object hierarchy and (2) the BlueZ services that occupy that hierarchy. In addition, we'll
// take that a step further by including the implementation right inside the description. Everything in one place.
//
// >>
// >>>  MANAGING SERVER DATA
// >>
//
// The purpose of the server is to serve data. Your application is responsible for providing that data to the server via two data
// accessors (a getter and a setter) that implemented in the form of delegates that are passed into the `ggkStart()` method.
//
// While the server is running, if data is updated via a write operation from the client the setter delegate will be called. If your
// application also generates or updates data periodically, it can push those updates to the server via call to
// `ggkNofifyUpdatedCharacteristic()` or `ggkNofifyUpdatedDescriptor()`.
//
// >>
// >>>  UNDERSTANDING THE UNDERLYING FRAMEWORKS
// >>
//
// The server description below attempts to provide a GATT-based interface in terms of GATT services, characteristics and
// descriptors. Consider the following sample:
//
//     .gattServiceBegin("text", "00000001-1E3C-FAD4-74E2-97A033F1BFAA")
//         .gattCharacteristicBegin("string", "00000002-1E3C-FAD4-74E2-97A033F1BFAA", {"read", "write", "notify"})
//
//             .onReadValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
//             {
//                 // Abbreviated for simplicity
//                 self.methodReturnValue(pInvocation, myTextString, true);
//             })
//
//             .onWriteValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
//             {
//                 // Abbreviated for simplicity
//                 myTextString = ...
//             })
//
//             .gattDescriptorBegin("description", "2901", {"read"})
//                 .onReadValue(DESCRIPTOR_METHOD_CALLBACK_LAMBDA
//                 {
//                     self.methodReturnValue(pInvocation, "Returns a test string", true);
//                 })
//
//             .gattDescriptorEnd()
//         .gattCharacteristicEnd()
//     .gattServiceEnd()
//
// The first thing you may notice abpout the sample is that all of the lines begin with a dot. This is because we're chaining
// methods together. Each method returns the appropriate type to provide context. For example, The `gattServiceBegin` method returns
// a reference to a `GattService` object which provides the proper context to create a characteristic within that service.
// Similarly, the `gattCharacteristicBegin` method returns a reference to a `GattCharacteristic` object which provides the proper
// context for responding to requests to read the characterisic value or add descriptors to the characteristic.
//
// For every `*Begin` method, there is a corresponding `*End` method, which returns us to the previous context. Indentation helps us
// keep track of where we are.
//
// Also note the use of the lambda macros, `CHARACTERISTIC_METHOD_CALLBACK_LAMBDA` and `DESCRIPTOR_METHOD_CALLBACK_LAMBDA`. These
// macros simplify the process of including our implementation directly in the description.
//
// The first parameter to each of the `*Begin` methods is a path node name. As we build our hierarchy, we give each node a name,
// which gets appended to it's parent's node (which in turns gets appended to its parent's node, etc.) If our root path was
// "/com/gobbledegook", then our service would have the path "/com/gobbledegook/text" and the characteristic would have the path
// "/com/gobbledegook/text/string", and the descriptor would have the path "/com/gobbledegook/text/string/description". These paths
// are important as they act like an addressing mechanism similar to paths on a filesystem or in a URL.
//
// The second parameter to each of the `*Begin` methods is a UUID as defined by the Bluetooth standard. These UUIDs effectively
// refer to an interface. You will see two different kinds of UUIDs: a short UUID ("2901") and a long UUID
// ("00000002-1E3C-FAD4-74E2-97A033F1BFAA").
//
// For more information on UUDSs, see GattUuid.cpp.
//
// In the example above, our non-standard UUIDs ("00000001-1E3C-FAD4-74E2-97A033F1BFAA") are something we generate ourselves. In the
// case above, we have created a custom service that simply stores a mutable text string. When the client enumerates our services
// they'll see this UUID and, assuming we've documented our interface behind this UUID for client authors, they can use our service
// to read and write a text string maintained on our server.
//
// The third parameter (which only applies to dharacteristics and descriptors) are a set of flags. You will find the current set of
// flags for characteristics and descriptors in the "BlueZ D-Bus GATT API description" at:
//
//     https://git.kernel.org/pub/scm/bluetooth/bluez.git/plain/doc/gatt-api.txt
//
// In addition to these structural methods, there are a small handful of helper methods for performing common operations. These
// helper methods are available within a method (such as `onReadValue`) through the use of a `self` reference. The `self` reference
// refers to the object at which the method is invoked (either a `GattCharacteristic` object or a `GattDescriptor` object.)
//
//     methodReturnValue and methodReturnVariant
//         These methods provide a means for returning values from Characteristics and Descriptors. The `-Value` form accept a set
//         of common types (int, string, etc.) If you need to provide a custom return type, you can do so by building your own
//         GVariant (which is a GLib construct) and using the `-Variant` form of the method.
//
//     sendChangeNotificationValue and sendChangeNotificationVariant
//         These methods provide a means for notifying changes for Characteristics. The `-Value` form accept a set of common types
//         (int, string, etc.) If you need to notify a custom return type, you can do so by building your own GVariant (which is a
//         GLib construct) and using the `-Variant` form of the method.
//
// For information about GVariants (what they are and how to work with them), see the GLib documentation at:
//
//     https://www.freedesktop.org/software/gstreamer-sdk/data/docs/latest/glib/glib-GVariantType.html
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include <algorithm>

#include "Server.h"
#include "ServerUtils.h"
#include "Utils.h"
#include "Globals.h"
#include "DBusObject.h"
#include "DBusInterface.h"
#include "GattProperty.h"
#include "GattService.h"
#include "GattUuid.h"
#include "GattCharacteristic.h"
#include "GattDescriptor.h"
#include "Logger.h"

namespace ggk {

// There's a good chance there will be a bunch of unused parameters from the lambda macros
#if defined(__GNUC__) && defined(__clang__)
	#pragma clang diagnostic push
	#pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#if defined(__GNUC__) && !defined(__clang__)
	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

// ---------------------------------------------------------------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------------------------------------------------------------

// Our one and only server. It's global.
std::shared_ptr<Server> TheServer = nullptr;

// ---------------------------------------------------------------------------------------------------------------------------------
// Object implementation
// ---------------------------------------------------------------------------------------------------------------------------------

// Our constructor builds our entire server description
//
// serviceName: The name of our server (collectino of services)
//
//     This is used to build the path for our Bluetooth services. It also provides the base for the D-Bus owned name (see
//     getOwnedName.)
//
//     This value will be stored as lower-case only.
//
//     Retrieve this value using the `getName()` method.
//
// advertisingName: The name for this controller, as advertised over LE
//
//     IMPORTANT: Setting the advertisingName will change the system-wide name of the device. If that's not what you want, set
//     BOTH advertisingName and advertisingShortName to as empty string ("") to prevent setting the advertising
//     name.
//
//     Retrieve this value using the `getAdvertisingName()` method.
//
// advertisingShortName: The short name for this controller, as advertised over LE
//
//     According to the spec, the short name is used in case the full name doesn't fit within Extended Inquiry Response (EIR) or
//     Advertising Data (AD).
//
//     IMPORTANT: Setting the advertisingName will change the system-wide name of the device. If that's not what you want, set
//     BOTH advertisingName and advertisingShortName to as empty string ("") to prevent setting the advertising
//     name.
//
//     Retrieve this value using the `getAdvertisingShortName()` method.
//
Server::Server(const std::string &serviceName, const std::string &advertisingName, const std::string &advertisingShortName, 
	GGKServerDataGetter getter, GGKServerDataSetter setter)
{
	// Save our names
	this->serviceName = serviceName;
	std::transform(this->serviceName.begin(), this->serviceName.end(), this->serviceName.begin(), ::tolower);
	this->advertisingName = advertisingName;
	this->advertisingShortName = advertisingShortName;

	// Register getter & setter for server data
	dataGetter = getter;
	dataSetter = setter;

	// Adapter configuration flags - set these flags based on how you want the adapter configured
	enableBREDR = false;
	enableSecureConnection = false;
	enableConnectable = true;
	enableDiscoverable = true;
	enableAdvertising = true;
	enableBondable = false;

	//
	// Define the server
	//

	// Create the root D-Bus object and push it into the list
	objects.push_back(DBusObject(DBusObjectPath() + "com" + getServiceName()));

	// We're going to build off of this object, so we need to get a reference to the instance of the object as it resides in the
	// list (and not the object that would be added to the list.)
	objects.back()

	// Service: Device Information (0xC76C00004F4340E7A27EADFBD971EC35)
	//
	// For some reason this is already defined so make it custom...
	//
	// See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.service.device_information.xml
	.gattServiceBegin("device", "C76C0000-4F43-40E7-A27E-ADFBD971EC35")

		// Characteristic: Manufacturer Name String (0x2A29)
		//
		// See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.manufacturer_name_string.xml
		.gattCharacteristicBegin("mfgr_name", "C76C2A29-4F43-40E7-A27E-ADFBD971EC35", {"read"})

			// This provides the value for the manufacturer.
			.onReadValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
			{
				self.methodReturnValue(pInvocation, "Senior Design Fall 2023", true);
			})

		.gattCharacteristicEnd()

		// Characteristic: Model Number String (0x2A24)
		//
		// See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.model_number_string.xml
		.gattCharacteristicBegin("model_num", "C76C2A24-4F43-40E7-A27E-ADFBD971EC35", {"read"})

			// This provides the device name.
			.onReadValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
			{
				self.methodReturnValue(pInvocation, "MWS", true);
			})

		.gattCharacteristicEnd()

	.gattServiceEnd()

	// Battery Service (0x180F)
	//
	// This is a fake battery service that conforms to org.bluetooth.service.battery_service. For details, see:
	//
	//     https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.service.battery_service.xml
	//
	// We also handle updates to the battery level from inside the server (see onUpdatedValue). There is an external method
	// (see main.cpp) that updates our battery level and posts an update using ggkPushUpdateQueue. Those updates are used
	// to notify us that our value has changed, which translates into a call to `onUpdatedValue` from the idleFunc (see
	// Init.cpp).
	.gattServiceBegin("battery", "180F")

		// Characteristic: Battery Level (0x2A19)
		//
		// See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.battery_level.xml
		.gattCharacteristicBegin("level", "2A19", {"read", "notify"})

			// Standard characteristic "ReadValue" method call
			.onReadValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
			{
				uint8_t batteryLevel = self.getDataValue<uint8_t>("battery/level", 0);
				self.methodReturnValue(pInvocation, batteryLevel, true);
			})

			// Handle updates to the battery level
			//
			// Here we use the onUpdatedValue to set a callback that isn't exposed to BlueZ, but rather allows us to manage
			// updates to our value. These updates may have come from our own server or some other source.
			//
			// We can handle updates in any way we wish, but the most common use is to send a change notification.
			.onUpdatedValue(CHARACTERISTIC_UPDATED_VALUE_CALLBACK_LAMBDA
			{
				uint8_t batteryLevel = self.getDataValue<uint8_t>("battery/level", 0);
				self.sendChangeNotificationValue(pConnection, batteryLevel);
				return true;
			})

		.gattCharacteristicEnd()
	.gattServiceEnd()

	/*
	 * Follower Service (0xCCA80000A7BE4E5A85069125CE3D98E8)
	 * 
	 * This contains the characteristics required to implement the following protocol,
	 * including sharing the signature used in the advertisement packet, the current 
	 * range (currently not variable), and a notification when the signature needs to 
	 * be changed based on the new challenge and response from the security service,
	 * as well as an acknowledgement for both sides.
	 * 
	 */
	.gattServiceBegin("follower", "CCA80000-A7BE-4E5A-8506-9125CE3D98E8")
		/*
		 * Is Activated Characteristic (0x0001)
		 *
		 * This represents whether the follower service is activated.
		 * 
		 */
		.gattCharacteristicBegin("is_activated", "CCA80001-A7BE-4E5A-8506-9125CE3D98E8", { "read", "write", "notify" })
			.onReadValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
            {
				bool isActivated = self.getDataValue<bool>("follower/is_activated", false);
				Logger::debug(SSTR << "Is Activated read: " << isActivated);
				self.methodReturnValue<bool>(pInvocation, isActivated, true);
			})
			.onWriteValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
            {
				// Parameter is uint 32-bit bool.
				GVariant* transmission = g_variant_get_child_value(pParameters, 0);
				gsize size;
				bool isActivated = (bool)*static_cast<const uint32_t*>(g_variant_get_fixed_array(transmission, &size, sizeof(uint32_t)));
				Logger::debug(SSTR << "Is Activated written: " << isActivated);

				// Set the value.
				self.setDataValue<bool>("follower/is_activated", isActivated);

				// Return properly by protocol.
                self.methodReturnVariant(pInvocation, NULL);
			})
			.onUpdatedValue(CHARACTERISTIC_UPDATED_VALUE_CALLBACK_LAMBDA
			{
				bool isActivated = self.getDataValue<bool>("follower/is_activated", false);
				Logger::debug(SSTR << "Is Activated updated: " << isActivated);
				self.sendChangeNotificationValue<bool>(pConnection, isActivated);
				return true;
			})
		.gattCharacteristicEnd()
		/*
		 * Signature Characteristic (0x0002)
		 *
		 * This represents the signature used when searching for the advertisement.
		 * 
		 */
		.gattCharacteristicBegin("signature", "CCA80002-A7BE-4E5A-8506-9125CE3D98E8", { "write" })
			.onWriteValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
            {
				// Convert to bytes of data.
				GVariant* transmission = g_variant_get_child_value(pParameters, 0);
				gsize size;
				const uint8_t* variantBytes = static_cast<const uint8_t*>(g_variant_get_fixed_array(transmission, &size, sizeof(uint8_t)));
				uint8_t* bytes = new uint8_t[size];
				for(int i = 0; i < (int)size; i++)
				{
					bytes[i] = variantBytes[i];
				}

				// Update the value through the setter.
				self.setDataPointer<uint8_t*>("follower/signature", bytes);
				std::string debugMessage = "Signature shared: ";
				for(int i = 0; i < (int)size; i++)
				{
					debugMessage += bytes[i];
				}
				Logger::debug(SSTR << debugMessage);

				// Return properly by protocol.
                self.methodReturnVariant(pInvocation, NULL);
			})
		.gattCharacteristicEnd()
		/*
		 * Change Advertisement Characteristic (0x0003)
		 * 
		 * This represents a notification of a request to change advertisements.
		 *  
		 */
		.gattCharacteristicBegin("change_advertisement", "CCA80003-A7BE-4E5A-8506-9125CE3D98E8", { "notify" })
			.onUpdatedValue(CHARACTERISTIC_UPDATED_VALUE_CALLBACK_LAMBDA
			{
				Logger::debug(SSTR << "Change Advertisement Notification sent");
				self.sendChangeNotificationValue<int>(pConnection, 1);
				return true;
			})
		.gattCharacteristicEnd()
		/*
		 * Range Characteristic (0x0004)
		 * 
		 * This represents the range from the user for the device. This is read-only for now.
		 *  
		 */
		.gattCharacteristicBegin("range", "CCA80004-A7BE-4E5A-8506-9125CE3D98E8", { "read", "write" })
			.onReadValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
            {
				double range = self.getDataValue<double>("follower/range", 5.0);
				Logger::debug(SSTR << "Range read: " << range);
				self.methodReturnValue(pInvocation, (uint8_t)(range * 2), true);
			})
            .onWriteValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
            {
				// Convert to bytes of data.
				GVariant* transmission = g_variant_get_child_value(pParameters, 0);
				gsize size;
				const uint8_t* variantBytes = static_cast<const uint8_t*>(g_variant_get_fixed_array(transmission, &size, sizeof(uint8_t)));
				uint8_t* bytes = new uint8_t[size];
				for(int i = 0; i < (int)size; i++)
				{
					bytes[i] = variantBytes[i];
				}

				// Update the value through the setter.
				self.setDataPointer<uint8_t*>("follower/range", bytes);
				std::string debugMessage = "Range Changed: ";
				for(int i = 0; i < (int)size; i++)
				{
					debugMessage += bytes[i];
				}
				Logger::debug(SSTR << debugMessage);

				// Return properly by protocol.
                self.methodReturnVariant(pInvocation, NULL);
			})
		.gattCharacteristicEnd()
		/*
		 * Acknowledge Characteristic (0x0005)
		 * 
		 * This represents a signal of acknowledgement from either party.
		 * 
		 */
		.gattCharacteristicBegin("acknowledge", "CCA80005-A7BE-4E5A-8506-9125CE3D98E8", { "write", "notify" })
			.onWriteValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
            {
				// Update the value through the setter.
				self.setDataValue<uint8_t>("follower/acknowledge", 1);
				Logger::debug(SSTR << "Acknowledge received");

				// Return properly by protocol.
                self.methodReturnVariant(pInvocation, NULL);
			})
			.onUpdatedValue(CHARACTERISTIC_UPDATED_VALUE_CALLBACK_LAMBDA
			{
				Logger::debug(SSTR << "Acknowledgement sent");
				self.sendChangeNotificationValue<int>(pConnection, 1);
				return true;
			})
		.gattCharacteristicEnd()
	.gattServiceEnd()

	/*
	 * Calibration Service (0x3C020000295C4EF086212E2AEA667070)
	 * 
	 * This contains the characteristics required to calibrate the MWS sensors on connection
	 * and by initiation of the user.
	 * 
	 */
	.gattServiceBegin("calibration", "3C020000-295C-4EF0-8621-2E2AEA667070")
		/*
		 * One Characteristic (0x0001)
		 * 
		 * This represents the value for calibration of sensor one.
		 * 
		 */
		.gattCharacteristicBegin("one", "3C020001-295C-4EF0-8621-2E2AEA667070", { "read", "write" })
			.onReadValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
			{
				uint32_t calibrationValue = self.getDataValue<uint32_t>("calibration/one", -69);
				Logger::debug(SSTR << "Calibration One read: " << calibrationValue);
				self.methodReturnValue(pInvocation, calibrationValue, true);
			})
			.onWriteValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
            {
				// Convert to bytes of data.
				GVariant* transmission = g_variant_get_child_value(pParameters, 0);
				gsize size;
				const uint8_t* variantBytes = static_cast<const uint8_t*>(g_variant_get_fixed_array(transmission, &size, sizeof(uint8_t)));
				uint8_t* bytes = new uint8_t[size];
				for(int i = 0; i < (int)size; i++)
				{
					bytes[i] = variantBytes[i];
				}

				// Update the value through the setter.
				self.setDataPointer<uint8_t*>("calibration/one", bytes);
				std::string debugMessage = "Calibration One Written: ";
				for(int i = 0; i < (int)size; i++)
				{
					debugMessage += bytes[i];
				}
				Logger::debug(SSTR << debugMessage);

				// Return properly by protocol.
                self.methodReturnVariant(pInvocation, NULL);
			})
		.gattCharacteristicEnd()
		/*
		 * Two Characteristic (0x0002)
		 * 
		 * This represents the value for calibration of sensor two.
		 * 
		 */
		.gattCharacteristicBegin("two", "3C020002-295C-4EF0-8621-2E2AEA667070", { "read", "write" })
			.onReadValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
			{
				uint32_t calibrationValue = self.getDataValue<uint32_t>("calibration/two", -69);
				Logger::debug(SSTR << "Calibration Two read: " << calibrationValue);
				self.methodReturnValue(pInvocation, calibrationValue, true);
			})
			.onWriteValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
            {
				// Convert to bytes of data.
				GVariant* transmission = g_variant_get_child_value(pParameters, 0);
				gsize size;
				const uint8_t* variantBytes = static_cast<const uint8_t*>(g_variant_get_fixed_array(transmission, &size, sizeof(uint8_t)));
				uint8_t* bytes = new uint8_t[size];
				for(int i = 0; i < (int)size; i++)
				{
					bytes[i] = variantBytes[i];
				}

				// Update the value through the setter.
				self.setDataPointer<uint8_t*>("calibration/two", bytes);
				std::string debugMessage = "Calibration Two Written: ";
				for(int i = 0; i < (int)size; i++)
				{
					debugMessage += bytes[i];
				}
				Logger::debug(SSTR << debugMessage);

				// Return properly by protocol.
                self.methodReturnVariant(pInvocation, NULL);
			})
		.gattCharacteristicEnd()
		/*
		 * Three Characteristic (0x0003)
		 * 
		 * This represents the value for calibration of sensor three.
		 * 
		 */
		.gattCharacteristicBegin("three", "3C020003-295C-4EF0-8621-2E2AEA667070", { "read", "write" })
			.onReadValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
			{
				uint32_t calibrationValue = self.getDataValue<uint32_t>("calibration/three", -69);
				Logger::debug(SSTR << "Calibration Three read: " << calibrationValue);
				self.methodReturnValue(pInvocation, calibrationValue, true);
			})
			.onWriteValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
            {
				// Convert to bytes of data.
				GVariant* transmission = g_variant_get_child_value(pParameters, 0);
				gsize size;
				const uint8_t* variantBytes = static_cast<const uint8_t*>(g_variant_get_fixed_array(transmission, &size, sizeof(uint8_t)));
				uint8_t* bytes = new uint8_t[size];
				for(int i = 0; i < (int)size; i++)
				{
					bytes[i] = variantBytes[i];
				}

				// Update the value through the setter.
				self.setDataPointer<uint8_t*>("calibration/three", bytes);
				std::string debugMessage = "Calibration Three Written: ";
				for(int i = 0; i < (int)size; i++)
				{
					debugMessage += bytes[i];
				}
				Logger::debug(SSTR << debugMessage);

				// Return properly by protocol.
                self.methodReturnVariant(pInvocation, NULL);
			})
		.gattCharacteristicEnd()
		/*
		 * Four Characteristic (0x0004)
		 * 
		 * This represents the value for calibration of sensor four.
		 * 
		 */
		.gattCharacteristicBegin("four", "3C020004-295C-4EF0-8621-2E2AEA667070", { "read", "write" })
			.onReadValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
			{
				uint32_t calibrationValue = self.getDataValue<uint32_t>("calibration/four", -69);
				Logger::debug(SSTR << "Calibration Four read: " << calibrationValue);
				self.methodReturnValue(pInvocation, calibrationValue, true);
			})
			.onWriteValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
            {
				// Convert to bytes of data.
				GVariant* transmission = g_variant_get_child_value(pParameters, 0);
				gsize size;
				const uint8_t* variantBytes = static_cast<const uint8_t*>(g_variant_get_fixed_array(transmission, &size, sizeof(uint8_t)));
				uint8_t* bytes = new uint8_t[size];
				for(int i = 0; i < (int)size; i++)
				{
					bytes[i] = variantBytes[i];
				}

				// Update the value through the setter.
				self.setDataPointer<uint8_t*>("calibration/four", bytes);
				std::string debugMessage = "Calibration Four Written: ";
				for(int i = 0; i < (int)size; i++)
				{
					debugMessage += bytes[i];
				}
				Logger::debug(SSTR << debugMessage);

				// Return properly by protocol.
                self.methodReturnVariant(pInvocation, NULL);
			})
		.gattCharacteristicEnd()
		/*
		 * Is Calibrating Characteristic (0x0005)
		 *
		 * This represents whether calibration is ongoing.
		 *
		 */
		.gattCharacteristicBegin("is_calibrating", "3C020005-295C-4EF0-8621-2E2AEA667070", { "write", "notify" })
			.onWriteValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
            {
				// Update the value through the setter.
				self.setDataValue<uint8_t>("calibration/is_calibrating", 1);
				Logger::debug(SSTR << "Beginning Calibration");

				// Return properly by protocol.
                self.methodReturnVariant(pInvocation, NULL);
			})
			.onUpdatedValue(CHARACTERISTIC_UPDATED_VALUE_CALLBACK_LAMBDA
			{
				Logger::debug(SSTR << "Ending Calibration");
				self.sendChangeNotificationValue<int>(pConnection, 1);
				return true;
			})
		.gattCharacteristicEnd()
		/*
		 * Target Characteristic (0x0006)
		 *
		 * This represents the target of calibration.
		 * 
		 */
		.gattCharacteristicBegin("target", "3C020006-295C-4EF0-8621-2E2AEA667070", { "write" })
			.onWriteValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
            {
				// Convert to bytes of data.
				GVariant* transmission = g_variant_get_child_value(pParameters, 0);
				gsize size;
				const uint8_t* variantBytes = static_cast<const uint8_t*>(g_variant_get_fixed_array(transmission, &size, sizeof(uint8_t)));
				uint8_t* bytes = new uint8_t[size];
				for(int i = 0; i < (int)size; i++)
				{
					bytes[i] = variantBytes[i];
				}

				// Update the value through the setter.
				self.setDataPointer<uint8_t*>("calibration/target", bytes);
				std::string debugMessage = "Target acquired: ";
				for(int i = 0; i < (int)size; i++)
				{
					debugMessage += bytes[i];
				}
				Logger::debug(SSTR << debugMessage);

				// Return properly by protocol.
                self.methodReturnVariant(pInvocation, NULL);
			})
		.gattCharacteristicEnd()
	.gattServiceEnd()

	/*
	 * Messenger Service (0xA58F00006AD94E688333BFEF89ADCF9B)
	 * 
	 * This contains the characteristics required to send a message to the user and 
	 * notify them of the existence of a message.
	 * 
	 */
	.gattServiceBegin("messenger", "A58F0000-6AD9-4E68-8333-BFEF89ADCF9B")
		/*
		 * Message Characteristic (0x0001)
		 *
		 * The message to give to the client.
		 *
		 */
		.gattCharacteristicBegin("message", "A58F0001-6AD9-4E68-8333-BFEF89ADCF9B", { "read", "notify" })
			.onReadValue(CHARACTERISTIC_METHOD_CALLBACK_LAMBDA
            {
				const char* message = self.getDataPointer<const char*>("messenger/message", "");
				Logger::debug(SSTR << "Message read: " << message);
				self.methodReturnValue(pInvocation, message, true);
			})
			.onUpdatedValue(CHARACTERISTIC_UPDATED_VALUE_CALLBACK_LAMBDA
			{
				const char* message = self.getDataPointer<const char*>("messenger/message", "");
				Logger::debug(SSTR << "Message updated: " << message);
				self.sendChangeNotificationValue(pConnection, message);
				return true;
			})
		.gattCharacteristicEnd()
	.gattServiceEnd()

	/*
	 * Security Service (0x)
	 * 
	 * 
	 */
	/*
	.gattServiceBegin("security", "")
		.gattCharacteristicBegin()
		.gattCharacteristicEnd()
	.gattServiceEnd()
	*/
	; // << -- NOTE THE SEMICOLON

	//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
	//                                                ____ _____ ___  _____
	//                                               / ___|_   _/ _ \|  _  |
	//                                               \___ \ | || | | | |_) |
	//                                                ___) || || |_| |  __/
	//                                               |____/ |_| \___/|_|
	//
	// You probably shouldn't mess with stuff beyond this point. It is required to meet BlueZ's requirements for a GATT Service.
	//
	// >>
	// >>  WHAT IT IS
	// >>
	//
	// From the BlueZ D-Bus GATT API description (https://git.kernel.org/pub/scm/bluetooth/bluez.git/plain/doc/gatt-api.txt):
	//
	//     "To make service registration simple, BlueZ requires that all objects that belong to a GATT service be grouped under a
	//     D-Bus Object Manager that solely manages the objects of that service. Hence, the standard DBus.ObjectManager interface
	//     must be available on the root service path."
	//
	// The code below does exactly that. Notice that we're doing much of the same work that our Server description does except that
	// instead of defining our own interfaces, we're following a pre-defined standard.
	//
	// The object types and method names used in the code below may look unfamiliar compared to what you're used to seeing in the
	// Server desecription. That's because the server description uses higher level types that define a more GATT-oriented framework
	// to build your GATT services. That higher level functionality was built using a set of lower-level D-Bus-oriented framework,
	// which is used in the code below.
	//  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

	// Create the root object and push it into the list. We're going to build off of this object, so we need to get a reference
	// to the instance of the object as it resides in the list (and not the object that would be added to the list.)
	//
	// This is a non-published object (as specified by the 'false' parameter in the DBusObject constructor.) This way, we can
	// include this within our server hieararchy (i.e., within the `objects` list) but it won't be exposed by BlueZ as a Bluetooth
	// service to clietns.
	objects.push_back(DBusObject(DBusObjectPath(), false));

	// Get a reference to the new object as it resides in the list
	DBusObject &objectManager = objects.back();

	// Create an interface of the standard type 'org.freedesktop.DBus.ObjectManager'
	//
	// See: https://dbus.freedesktop.org/doc/dbus-specification.html#standard-interfaces-objectmanager
	auto omInterface = std::make_shared<DBusInterface>(objectManager, "org.freedesktop.DBus.ObjectManager");

	// Add the interface to the object manager
	objectManager.addInterface(omInterface);

	// Finally, we setup the interface. We do this by adding the `GetManagedObjects` method as specified by D-Bus for the
	// 'org.freedesktop.DBus.ObjectManager' interface.
	const char *pInArgs[] = { nullptr };
	const char *pOutArgs = "a{oa{sa{sv}}}";
	omInterface->addMethod("GetManagedObjects", pInArgs, pOutArgs, INTERFACE_METHOD_CALLBACK_LAMBDA
	{
		ServerUtils::getManagedObjects(pInvocation);
	});
}

// ---------------------------------------------------------------------------------------------------------------------------------
// Utilitarian
// ---------------------------------------------------------------------------------------------------------------------------------

// Find a D-Bus interface within the given D-Bus object
//
// If the interface was found, it is returned, otherwise nullptr is returned
std::shared_ptr<const DBusInterface> Server::findInterface(const DBusObjectPath &objectPath, const std::string &interfaceName) const
{
	for (const DBusObject &object : objects)
	{
		std::shared_ptr<const DBusInterface> pInterface = object.findInterface(objectPath, interfaceName);
		if (pInterface != nullptr)
		{
			return pInterface;
		}
	}

	return nullptr;
}

// Find and call a D-Bus method within the given D-Bus object on the given D-Bus interface
//
// If the method was called, this method returns true, otherwise false. There is no result from the method call itself.
bool Server::callMethod(const DBusObjectPath &objectPath, const std::string &interfaceName, const std::string &methodName, GDBusConnection *pConnection, GVariant *pParameters, GDBusMethodInvocation *pInvocation, gpointer pUserData) const
{
	for (const DBusObject &object : objects)
	{
		if (object.callMethod(objectPath, interfaceName, methodName, pConnection, pParameters, pInvocation, pUserData))
		{
			return true;
		}
	}

	return false;
}

// Find a GATT Property within the given D-Bus object on the given D-Bus interface
//
// If the property was found, it is returned, otherwise nullptr is returned
const GattProperty *Server::findProperty(const DBusObjectPath &objectPath, const std::string &interfaceName, const std::string &propertyName) const
{
	std::shared_ptr<const DBusInterface> pInterface = findInterface(objectPath, interfaceName);

	// Try each of the GattInterface types that support properties?
	if (std::shared_ptr<const GattInterface> pGattInterface = TRY_GET_CONST_INTERFACE_OF_TYPE(pInterface, GattInterface))
	{
		return pGattInterface->findProperty(propertyName);
	}
	else if (std::shared_ptr<const GattService> pGattInterface = TRY_GET_CONST_INTERFACE_OF_TYPE(pInterface, GattService))
	{
		return pGattInterface->findProperty(propertyName);
	}
	else if (std::shared_ptr<const GattCharacteristic> pGattInterface = TRY_GET_CONST_INTERFACE_OF_TYPE(pInterface, GattCharacteristic))
	{
		return pGattInterface->findProperty(propertyName);
	}

	return nullptr;
}

}; // namespace ggk