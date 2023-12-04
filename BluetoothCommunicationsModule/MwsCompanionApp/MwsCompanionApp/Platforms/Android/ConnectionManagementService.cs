using Android.Bluetooth.LE;
using Android.Bluetooth;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Android.Runtime;
using Android.App.Roles;
using static Java.Util.Jar.Attributes;
using Javax.Security.Auth;
using MwsCompanionApp.Objects;
using Android.OS;
using MwsCompanionApp.Platforms.Android.Handlers;
using Java.Util;
using Android.Systems;
using Android.Content.PM;

namespace MwsCompanionApp.Services
{
    public partial class ConnectionManagementService
    {
        /// <summary>
        /// The UUId of the CCCD.
        /// </summary>
        private const string _cccdUuid = "00002902-0000-1000-8000-00805F9B34FB";

        /// <summary>
        /// The currently connected device.
        /// </summary>
        private BluetoothGatt _connectedDevice;

        /// <summary>
        /// The services avaiable.
        /// </summary>
        private ServicesContainer _services;

        private partial bool IsConnected() 
        {
            return this._connectedDevice != null;
        }

        /// <summary>
        /// The current data being written.
        /// </summary>
        private byte[] _currentWrite;

        /// <summary>
        /// Whether writing is ongoing.
        /// </summary>
        private bool _isWriting;

        /// <summary>
        /// The callback for the current advertising set.
        /// </summary>
        private GenericAdvertisingSetCallback _currentAdvertiseCallback;

        /// <summary>
        /// Creates a connection management service with the given services.
        /// </summary>
        /// <param name="services">
        /// The services available to this service.
        /// </param>
        public ConnectionManagementService(ServicesContainer services)
        {
            this._services = services;
            this._connectedDevice = null;
            this._isConnectionSetUp = false;
            this._hasManufacturerBeenConfirmed = false;
            this._hasModelBeenConfirmed = false;
            this._currentWrite = null;
            this._isWriting = false;
            this._currentAdvertiseCallback = null;
        }

        private partial async Task SendConnectionRequest()
        {
            // Get bluetooth adapter.
            BluetoothManager bleManager = (BluetoothManager)MainActivity.Instance.GetSystemService(MainActivity.BluetoothService);
            if(bleManager != null)
            {
                // Keeps track of the characteristics that need notifications enabled.
                List<BluetoothGattDescriptor> characteristicsWithNotifications = new List<BluetoothGattDescriptor>();

                // The number of delays to wait until quitting.
                int numberOfTries = 10;

                // Whether scanning is ongoing.
                bool isScanning = true;

                // Get advertisement interface.
                BluetoothLeScanner scanner = bleManager.Adapter.BluetoothLeScanner;

                // Begin scanning.
                scanner.StartScan(new List<ScanFilter>()
                                  {
                                      new ScanFilter.Builder()
                                                    .SetServiceUuid(MwsUuidObjects.AdvertiseUuid)
                                                    .SetDeviceName(this._currentConnection.Name)
                                                    .Build()
                                  },
                                  new ScanSettings.Builder().Build(),
                                  new GenericScanCallback() 
                                  { 
                                      OnScanResultCallback = (callbackType, result) => 
                                      {
                                          if(this._connectedDevice == null && // Only get first result.
                                             numberOfTries > 0 && // Quit if tries are exhausted.
                                             isScanning) // Quit if this block has been entered.
                                          {
                                              isScanning = false;
                                              scanner.StopScan(new GenericScanCallback());
                                              this._connectedDevice = result.Device.ConnectGatt(MainActivity.Instance,
                                                                false,
                                                                new GenericBluetoothGattCallback() 
                                                                { 
                                                                    OnConnectionStateChangedCallback = (gatt, status, newStatus) => 
                                                                    {
                                                                        if(newStatus == ProfileState.Disconnected ||
                                                                            newStatus == ProfileState.Disconnecting)
                                                                        {
                                                                            // Handle the disconnection.
                                                                            this.OnDisconnect();
                                                                            this._connectedDevice.Close();
                                                                            this._connectedDevice = null;
                                                                            this._currentWrite = null;
                                                                            this._isWriting = false;
                                                                            if(this._currentAdvertiseCallback != null) 
                                                                            {
                                                                                this.StopAdvertisingLocation();
                                                                            }
                                                                        }
                                                                        else if(status != GattStatus.Success)
                                                                        {
                                                                            // Disconnect if there is a problem.
                                                                            this._connectedDevice.Disconnect();
                                                                            this._services.EventService.InvokeSystemNotificationDispatchedEvent(this, 
                                                                                new Interfaces.SystemNotificationEventArgs("Disconnected", 
                                                                                                                            "Connection to the MWS dropped.", 
                                                                                                                            subtitle: "Connection Status", 
                                                                                                                            category: Interfaces.NotificationCategory.Error, 
                                                                                                                            isUrgent: true));
                                                                        }
                                                                        else 
                                                                        {
                                                                            // Figure out if this is actually an MWS.
                                                                            App.Current.Dispatcher.Dispatch(() => this._connectedDevice.DiscoverServices());
                                                                        }
                                                                    },
                                                                    OnServicesDiscoveredCallback = async (gatt, status) => 
                                                                    {
                                                                        // Find the device service.
                                                                        BluetoothGattService deviceService = gatt.GetService(MwsUuidObjects.DeviceServiceUuid);

                                                                        // Check the model number and manufacturer.
                                                                        BluetoothGattCharacteristic manufacturerCharacteristic = deviceService.GetCharacteristic(MwsUuidObjects.DeviceManufacturerUuid);
                                                                        BluetoothGattCharacteristic modelCharacteristic = deviceService.GetCharacteristic(MwsUuidObjects.DeviceModelUuid);
                                                                        if((manufacturerCharacteristic.Properties & GattProperty.Read) != 0 &&
                                                                           (modelCharacteristic.Properties & GattProperty.Read) != 0)
                                                                        {
                                                                            // Read each characteristic to check if valid.
                                                                            gatt.ReadCharacteristic(manufacturerCharacteristic);

                                                                            // Check that the required services and characteristics are present and set up notifications.
                                                                            if(gatt.GetService(MwsUuidObjects.FollowerServiceUuid) is BluetoothGattService followerService &&
                                                                               gatt.GetService(MwsUuidObjects.MessengerServiceUuid) is BluetoothGattService messengerService &&
                                                                               gatt.GetService(MwsUuidObjects.CalibrationServiceUuid) is BluetoothGattService calibrationService &&
                                                                               followerService.GetCharacteristic(MwsUuidObjects.FollowerAcknowledgeUuid) is BluetoothGattCharacteristic followerAcknowledge &&
                                                                               followerService.GetCharacteristic(MwsUuidObjects.FollowerChangeAdvertisementUuid) is BluetoothGattCharacteristic followerChangeAdvertisement &&
                                                                               followerService.GetCharacteristic(MwsUuidObjects.FollowerIsActivatedUuid) is BluetoothGattCharacteristic followerIsActivated &&
                                                                               followerService.GetCharacteristic(MwsUuidObjects.FollowerRangeUuid) is BluetoothGattCharacteristic followerRange &&
                                                                               followerService.GetCharacteristic(MwsUuidObjects.FollowerSignatureUuid) is BluetoothGattCharacteristic followerSignature &&
                                                                               messengerService.GetCharacteristic(MwsUuidObjects.MessengerMessageUuid) is BluetoothGattCharacteristic messengerMessage &&
                                                                               calibrationService.GetCharacteristic(MwsUuidObjects.CalibrationOneUuid) is BluetoothGattCharacteristic calibrationOne &&
                                                                               calibrationService.GetCharacteristic(MwsUuidObjects.CalibrationTwoUuid) is BluetoothGattCharacteristic calibrationTwo &&
                                                                               calibrationService.GetCharacteristic(MwsUuidObjects.CalibrationThreeUuid) is BluetoothGattCharacteristic calibrationThree &&
                                                                               calibrationService.GetCharacteristic(MwsUuidObjects.CalibrationFourUuid) is BluetoothGattCharacteristic calibrationFour &&
                                                                               calibrationService.GetCharacteristic(MwsUuidObjects.CalibrationIsCalibratingUuid) is BluetoothGattCharacteristic calibrationIsCalibrating &&
                                                                               calibrationService.GetCharacteristic(MwsUuidObjects.CalibrationTargetUuid) is BluetoothGattCharacteristic calibrationTarget &&
                                                                               (followerAcknowledge.Properties & (GattProperty.Write | GattProperty.Notify)) != 0 &&
                                                                               (followerChangeAdvertisement.Properties & GattProperty.Notify) != 0 &&
                                                                               (followerIsActivated.Properties & (GattProperty.Read | GattProperty.Write | GattProperty.Notify)) != 0 &&
                                                                               (followerRange.Properties & (GattProperty.Read | GattProperty.Write)) != 0 &&
                                                                               (followerSignature.Properties & GattProperty.Write) != 0 &&
                                                                               (messengerMessage.Properties & (GattProperty.Read | GattProperty.Notify)) != 0 &&
                                                                               (calibrationOne.Properties & (GattProperty.Write | GattProperty.Read)) != 0 &&
                                                                               (calibrationTwo.Properties & (GattProperty.Write | GattProperty.Read)) != 0 &&
                                                                               (calibrationThree.Properties & (GattProperty.Write | GattProperty.Read)) != 0 &&
                                                                               (calibrationFour.Properties & (GattProperty.Write | GattProperty.Read)) != 0 &&
                                                                               (calibrationIsCalibrating.Properties & (GattProperty.Write | GattProperty.Notify)) != 0 &&
                                                                               (calibrationTarget.Properties & GattProperty.Write) != 0)
                                                                            {
                                                                                // Set up notifications for these properties.
                                                                                UUID cccdUuid = UUID.FromString(ConnectionManagementService._cccdUuid);
                                                                                characteristicsWithNotifications.Add(followerChangeAdvertisement.GetDescriptor(cccdUuid));
                                                                                characteristicsWithNotifications.Add(followerIsActivated.GetDescriptor(cccdUuid));
                                                                                characteristicsWithNotifications.Add(messengerMessage.GetDescriptor(cccdUuid));
                                                                                characteristicsWithNotifications.Add(calibrationIsCalibrating.GetDescriptor(cccdUuid));
                                                                                gatt.SetCharacteristicNotification(followerAcknowledge, true);
                                                                                gatt.SetCharacteristicNotification(followerChangeAdvertisement, true);
                                                                                gatt.SetCharacteristicNotification(followerIsActivated, true);
                                                                                gatt.SetCharacteristicNotification(messengerMessage, true);
                                                                                gatt.SetCharacteristicNotification(calibrationIsCalibrating, true);

                                                                                await Task.Delay(TimeSpan.FromSeconds(1));

                                                                                // Write to the CCCDs to enable notifications.
                                                                                BluetoothGattDescriptor cccd = followerAcknowledge.GetDescriptor(cccdUuid);
                                                                                if(OperatingSystem.IsAndroidVersionAtLeast(33))
                                                                                {
                                                                                    gatt.WriteDescriptor(cccd, BluetoothGattDescriptor.EnableNotificationValue.ToArray());
                                                                                }
                                                                                else
                                                                                {
                                                                                    cccd.SetValue(BluetoothGattDescriptor.EnableNotificationValue.ToArray());
                                                                                    gatt.WriteDescriptor(cccd);
                                                                                }

                                                                                this._isConnectionSetUp = true;
                                                                            }
                                                                            else 
                                                                            {
                                                                                // This is not a valid MWS.
                                                                                this.DisconnectForInvalidDevice();
                                                                            }
                                                                        }
                                                                        else 
                                                                        {
                                                                            // Disconnect since this is obviously not an MWS.
                                                                            this.DisconnectForInvalidDevice();
                                                                        }
                                                                    },
                                                                    OnCharacteristicReadCallback = this.ReadCharacteristic,
                                                                    OnCharacteristicReadLegacyCallback = (gatt, characteristic, status) =>
                                                                    {
                                                                        if(!OperatingSystem.IsAndroidVersionAtLeast(33))
                                                                        {
                                                                            this.ReadCharacteristic(gatt, characteristic, characteristic.GetValue(), status);
                                                                        }
                                                                    },
                                                                    OnCharacteristicWriteCallback = (gatt, characteristic, status) => 
                                                                    {
                                                                        if(status == GattStatus.Failure ||
                                                                           status == GattStatus.ConnectionCongested)
                                                                        {
                                                                            // Retry the write.
                                                                            if(OperatingSystem.IsAndroidVersionAtLeast(33))
                                                                            {
                                                                                gatt.WriteCharacteristic(characteristic, this._currentWrite, (int)GattWriteType.Default);
                                                                            }
                                                                            else 
                                                                            {
                                                                                characteristic.WriteType = GattWriteType.Default;
                                                                                characteristic.SetValue(this._currentWrite);
                                                                                gatt.WriteCharacteristic(characteristic);
                                                                            }
                                                                        }
                                                                        else
                                                                        {
                                                                            // End the writing process.
                                                                            this._currentWrite = null;
                                                                            this._isWriting = false;
                                                                        }
                                                                    },
                                                                    OnCharacteristicChangedCallback = (gatt, characteristic, value) =>
                                                                    {
                                                                        this.ProcessNotification(characteristic.Uuid.ToString(), value);
                                                                    },
                                                                    OnCharacteristicChangedLegacyCallback = (gatt, characteristic) =>
                                                                    {
                                                                        if(!OperatingSystem.IsAndroidVersionAtLeast(33))
                                                                        {
                                                                            this.ProcessNotification(characteristic.Uuid.ToString(), characteristic.GetValue());
                                                                        }
                                                                    },
                                                                    OnDescriptorWriteCallback = (gatt, descriptor, status) => 
                                                                    {
                                                                        App.Current.Dispatcher.Dispatch(() =>
                                                                        {
                                                                            if(descriptor.Uuid.ToString().ToUpper() == ConnectionManagementService._cccdUuid &&
                                                                               characteristicsWithNotifications.Any())
                                                                            {
                                                                                // Enable notifications.
                                                                                BluetoothGattDescriptor cccd = characteristicsWithNotifications.ElementAt(0);
                                                                                characteristicsWithNotifications.RemoveAt(0);
                                                                                if(OperatingSystem.IsAndroidVersionAtLeast(33))
                                                                                {
                                                                                    gatt.WriteDescriptor(cccd, BluetoothGattDescriptor.EnableNotificationValue.ToArray());
                                                                                }
                                                                                else
                                                                                {
                                                                                    cccd.SetValue(BluetoothGattDescriptor.EnableNotificationValue.ToArray());
                                                                                    gatt.WriteDescriptor(cccd);
                                                                                }
                                                                            }
                                                                        });
                                                                    }
                                                                });
                                          }
                                          else if(numberOfTries <= 0 &&
                                                  isScanning) 
                                          {
                                              scanner.StopScan(new GenericScanCallback());
                                          }
                                      }
                                  });

                // Wait for connection.
                while(numberOfTries > 0 &&
                      isScanning) 
                {
                    await Task.Delay(TimeSpan.FromSeconds(5));
                }
            }
        }

        private partial void SendDisconnectionRequest()
        {
            this._connectedDevice.Disconnect();
        }

        private partial void ReadValue(string serviceUuid, string characteristicUuid)
        {
            BluetoothGattService service = this._connectedDevice.GetService(UUID.FromString(serviceUuid));
            BluetoothGattCharacteristic characteristic = service.GetCharacteristic(UUID.FromString(characteristicUuid));
            this._connectedDevice.ReadCharacteristic(characteristic);
        }

        private partial async Task WriteValue(string serviceUuid, string characteristicUuid, byte[] value)
        {
            this._isWriting = true;
            this._currentWrite = value;
            BluetoothGattService service = this._connectedDevice.GetService(UUID.FromString(serviceUuid));
            BluetoothGattCharacteristic characteristic = service.GetCharacteristic(UUID.FromString(characteristicUuid));
            if(OperatingSystem.IsAndroidVersionAtLeast(33))
            {
                this._connectedDevice.WriteCharacteristic(characteristic, value, (int)GattWriteType.Default);
            }
            else
            {
                characteristic.SetValue(value);
                characteristic.WriteType = GattWriteType.Default;
                this._connectedDevice.WriteCharacteristic(characteristic);
            }

            // Wait for the write to be complete.
            while(this._isWriting) 
            {
                await Task.Delay(TimeSpan.FromSeconds(1));
            }
        }

        /// <summary>
        /// The callback for reading any characteristic.
        /// </summary>
        /// <param name="gatt">
        /// The bluetooth gatt server object.
        /// </param>
        /// <param name="characteristic">
        /// The characteristic that was read.
        /// </param>
        /// <param name="data">
        /// The value read from the characteristic.
        /// </param>
        /// <param name="status">
        /// The status of the GATT server from the operation.
        /// </param>
        private void ReadCharacteristic(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, byte[] data, GattStatus status)
        {
            // Verify the read was successful.
            if(status == GattStatus.Success)
            {
                this.ReadValueCallback(characteristic.Uuid.ToString(), data);
            }
            else if(status == GattStatus.ConnectionCongested ||
                    status == GattStatus.Failure)
            {
                // The read should be retried.
                gatt.ReadCharacteristic(characteristic);
            }
            else 
            {
                // The device does not fit the interface of an MWS.
                this.DisconnectForInvalidDevice();
            }
        }

        private partial bool SendRenameRequest(string name) 
        {
            return true;
        }

        private partial byte[] GenerateSignature()
        {
            BluetoothManager bleManager = (BluetoothManager)MainActivity.Instance.GetSystemService(MainActivity.BluetoothService);
            if(bleManager == null)
            {
                return null;
            }
            else
            {
                // Get the bytes as a C-string.
                return Encoding.ASCII.GetBytes(bleManager.Adapter.Name + '\0');
            }
        }

        private partial bool StartAdvertisingLocation() 
        {
            // Get bluetooth adapter.
            BluetoothManager bleManager = (BluetoothManager)MainActivity.Instance.GetSystemService(MainActivity.BluetoothService);
            if(bleManager == null) 
            { 
                return false; 
            }
            else
            {
                // Get advertisement interface.
                BluetoothLeAdvertiser advertiser = bleManager.Adapter.BluetoothLeAdvertiser;

                // Build the advertisement and start advertising.
                AdvertisingSetParameters advertisingParameters = (new AdvertisingSetParameters.Builder())
                                                                  .SetLegacyMode(true)
                                                                  .SetScannable(true)
                                                                  .SetConnectable(false)
                                                                  .SetInterval(AdvertisingSetParameters.IntervalLow)
                                                                  .SetTxPowerLevel(AdvertiseTxPower.High)
                                                                  .Build();
                AdvertiseData advertiseData = (new AdvertiseData.Builder())
                                               .SetIncludeDeviceName(true)
                                               .SetIncludeTxPowerLevel(true)
                                               .AddServiceUuid(MwsUuidObjects.ClientAdvertisementUuid)
                                               .Build();
                this._currentAdvertiseCallback = new GenericAdvertisingSetCallback();
                advertiser.StartAdvertisingSet(advertisingParameters, advertiseData, null, null, null, this._currentAdvertiseCallback);
                return true;
            }
        }

        private partial void StopAdvertisingLocation() 
        {
            // Stop the advertisement.
            BluetoothManager bleManager = (BluetoothManager)MainActivity.Instance.GetSystemService(MainActivity.BluetoothService);
            BluetoothLeAdvertiser advertiser = bleManager.Adapter.BluetoothLeAdvertiser;
            advertiser?.StopAdvertisingSet(this._currentAdvertiseCallback);
            this._currentAdvertiseCallback = null;
        }
    }

    /// <summary>
    /// Contains the UUIDs of the services and characteristics.
    /// </summary>
    internal static class MwsUuidObjects
    {
        /// <summary>
        /// The UUID used by the MWS for advertising.
        /// </summary>
        public static readonly ParcelUuid AdvertiseUuid = ParcelUuid.FromString(MwsUuidStrings.AdvertiseUuid);

        /// <summary>
        /// The UUID of the device service.
        /// </summary>
        public static readonly UUID DeviceServiceUuid = UUID.FromString(MwsUuidStrings.DeviceServiceUuid);

        /// <summary>
        /// The UUID of the mfgr_name characteristic of the device service.
        /// </summary>
        public static readonly UUID DeviceManufacturerUuid = UUID.FromString(MwsUuidStrings.DeviceManufacturerUuid);

        /// <summary>
        /// The UUID of the model_num characteristic of the device service.
        /// </summary>
        public static readonly UUID DeviceModelUuid = UUID.FromString(MwsUuidStrings.DeviceModelUuid);

        /// <summary>
        /// The UUID of the battery service.
        /// </summary>
        public static readonly UUID BatteryServiceUuid = UUID.FromString(MwsUuidStrings.BatteryServiceUuid);

        /// <summary>
        /// The UUID of the level characteristic of the battery service.
        /// </summary>
        public static readonly UUID BatteryLevelUuid = UUID.FromString(MwsUuidStrings.BatteryLevelUuid);

        /// <summary>
        /// The UUID of the follower service.
        /// </summary>
        public static readonly UUID FollowerServiceUuid = UUID.FromString(MwsUuidStrings.FollowerServiceUuid);

        /// <summary>
        /// The UUID of the is_activated characteristic of the follower service.
        /// </summary>
        public static readonly UUID FollowerIsActivatedUuid = UUID.FromString(MwsUuidStrings.FollowerIsActivatedUuid);

        /// <summary>
        /// The UUID of the signature characteristic of the follower service.
        /// </summary>
        public static readonly UUID FollowerSignatureUuid = UUID.FromString(MwsUuidStrings.FollowerSignatureUuid);

        /// <summary>
        /// The UUID of the change_advertisement characteristic of the follower service.
        /// </summary>
        public static readonly UUID FollowerChangeAdvertisementUuid = UUID.FromString(MwsUuidStrings.FollowerChangeAdvertisementUuid);

        /// <summary>
        /// The UUID of the range characteristic of the follower service.
        /// </summary>
        public static readonly UUID FollowerRangeUuid = UUID.FromString(MwsUuidStrings.FollowerRangeUuid);

        /// <summary>
        /// The UUID of the acknowledge characteristic of the follower service.
        /// </summary>
        public static readonly UUID FollowerAcknowledgeUuid = UUID.FromString(MwsUuidStrings.FollowerAcknowledgeUuid);

        /// <summary>
        /// The UUID of the messenger service.
        /// </summary>
        public static readonly UUID MessengerServiceUuid = UUID.FromString(MwsUuidStrings.MessengerServiceUuid);

        /// <summary>
        /// The UUID of the message characteristic of the messenger service.
        /// </summary>
        public static readonly UUID MessengerMessageUuid = UUID.FromString(MwsUuidStrings.MessengerMessageUuid);

        /// <summary>
        /// The UUID advertised by the client.
        /// </summary>
        public static readonly ParcelUuid ClientAdvertisementUuid = ParcelUuid.FromString(MwsUuidStrings.ClientAdvertisementUuid);

        /// <summary>
        /// The UUID of the calibration service.
        /// </summary>
        public static readonly UUID CalibrationServiceUuid = UUID.FromString(MwsUuidStrings.CalibrationServiceUuid);

        /// <summary>
        /// The UUID of the value for number One.
        /// </summary>
        public static readonly UUID CalibrationOneUuid = UUID.FromString(MwsUuidStrings.CalibrationOneUuid);

        /// <summary>
        /// The UUID of the value for number Two.
        /// </summary>
        public static readonly UUID CalibrationTwoUuid = UUID.FromString(MwsUuidStrings.CalibrationTwoUuid);

        /// <summary>
        /// The UUID of the value for number Three.
        /// </summary>
        public static readonly UUID CalibrationThreeUuid = UUID.FromString(MwsUuidStrings.CalibrationThreeUuid);

        /// <summary>
        /// The UUID of the value for number Four.
        /// </summary>
        public static readonly UUID CalibrationFourUuid = UUID.FromString(MwsUuidStrings.CalibrationFourUuid);

        /// <summary>
        /// The UUID for whether calibration is occurring.
        /// </summary>
        public static readonly UUID CalibrationIsCalibratingUuid = UUID.FromString(MwsUuidStrings.CalibrationIsCalibratingUuid);

        /// <summary>
        /// The UUID for the target to calibrate.
        /// </summary>
        public static readonly UUID CalibrationTargetUuid = UUID.FromString(MwsUuidStrings.CalibrationTargetUuid);
    }
}
