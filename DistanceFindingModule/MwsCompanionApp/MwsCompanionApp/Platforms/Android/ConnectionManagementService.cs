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

namespace MwsCompanionApp.Services
{
    public partial class ConnectionManagementService
    {
        /// <summary>
        /// The currently connected device.
        /// </summary>
        private BluetoothGatt _connectedDevice;

        /// <summary>
        /// The services avaiable.
        /// </summary>
        private ServicesContainer _services;

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
        }

        private partial async Task<bool> SendConnectionRequest()
        {
            // Get bluetooth adapter.
            BluetoothManager bleManager = (BluetoothManager)MainActivity.Instance.GetSystemService(MainActivity.BluetoothService);
            if(bleManager == null)
            { 
                return false;
            }
            else
            {
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
                                                    .SetServiceUuid(ParcelUuid.FromString("CCA85698-A7BE-4E5A-8506-9125CE3D98E8"))
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
                                                                                                            this.OnDisconnect();
                                                                                                            this._connectedDevice.Close();
                                                                                                            this._connectedDevice = null;
                                                                                                        }
                                                                                                    }
                                                                                                });
                                          }
                                          else if(numberOfTries <= 0 &&
                                                  isScanning) 
                                          {
                                              scanner.StopScan(new GenericScanCallback());
                                          }
                                      },
                                      OnBatchScanResultsCallback = null,
                                      OnScanFailedCallback = null
                                  });

                // Wait to update UI.
                while(numberOfTries > 0 &&
                      isScanning) 
                {
                    await Task.Delay(TimeSpan.FromSeconds(5));
                }

                return this._connectedDevice != null;
            }
        }

        private partial void SendDisconnectionRequest()
        {
            this._connectedDevice.Disconnect();
        }

        private partial bool SendRenameRequest(string name) 
        {
            return true;
        }

        private partial bool SetupLocationSharing() 
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
                                               .Build();
                advertiser.StartAdvertisingSet(advertisingParameters, advertiseData, null, null, null, LocationSharingAdvertisementEventCallback.Instance);
                return true;
            }
        }

        private partial void EndLocationSharing() 
        {
            // Stop the advertisement.
            BluetoothManager bleManager = (BluetoothManager)MainActivity.Instance.GetSystemService(MainActivity.BluetoothService);
            BluetoothLeAdvertiser advertiser = bleManager.Adapter.BluetoothLeAdvertiser;
            advertiser?.StopAdvertisingSet(LocationSharingAdvertisementEventCallback.Instance);
        }

        /// <summary>
        /// Represents the set of callbacks used for advertising events.
        /// </summary>
        private class LocationSharingAdvertisementEventCallback : AdvertisingSetCallback
        {
            /// <summary>
            /// The original name of the device.
            /// </summary>
            private string OriginalName { get; set; }

            /// <summary>
            /// The singular instance of the callback.
            /// </summary>
            public static LocationSharingAdvertisementEventCallback Instance = new LocationSharingAdvertisementEventCallback();

            /// <summary>
            /// Creates a callback for location sharing.
            /// </summary>
            public LocationSharingAdvertisementEventCallback() 
            {
                this.OriginalName = ((BluetoothManager)MainActivity.Instance.GetSystemService(MainActivity.BluetoothService))?.Adapter.Name;
            }

            /// <inheritdoc/>
            public override void OnAdvertisingSetStarted(AdvertisingSet advertisingSet, int txPower, [GeneratedEnum] AdvertiseResult status)
            {
                // Nothing needs to be done.
            }

            /// <inheritdoc/>
            public override void OnAdvertisingDataSet(AdvertisingSet advertisingSet, [GeneratedEnum] AdvertiseResult status)
            {
                // Nothing needs to be done.
            }

            /// <inheritdoc/>
            public override void OnScanResponseDataSet(AdvertisingSet advertisingSet, [GeneratedEnum] AdvertiseResult status)
            {
                // Nothing needs to be done.
            }

            /// <inheritdoc/>
            public override void OnAdvertisingSetStopped(AdvertisingSet advertisingSet)
            {
                // Reset the adapter to the original name.
                ((BluetoothManager)MainActivity.Instance.GetSystemService(MainActivity.BluetoothService)).Adapter.SetName(this.OriginalName);
            }
        }
    }
}
