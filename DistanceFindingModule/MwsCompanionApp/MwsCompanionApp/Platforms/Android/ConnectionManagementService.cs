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

namespace MwsCompanionApp.Services
{
    public partial class ConnectionManagementService
    {
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
        }

        private partial bool SendConnectionRequest() 
        {
            return true;
        }

        private partial void SendDisconnectionRequest()
        {

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
                bleManager.Adapter.SetName("MWS-" + bleManager.Adapter.Name);
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
