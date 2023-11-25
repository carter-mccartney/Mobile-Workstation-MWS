using Android;
using Android.Bluetooth;
using Android.Bluetooth.LE;
using Android.Content;
using Android.OS;
using Android.Runtime;
using Android.Systems;
using AndroidX.Core.Content;
using MwsCompanionApp.Interfaces;
using MwsCompanionApp.Objects;
using MwsCompanionApp.Platforms.Android.Handlers;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Services
{
    public partial class ScanningService
    {
        public ScanningService(ServicesContainer services)
        {
            this._services = services;
            this.AvailableConnections = new HashSet<Mws>();
        }

        private partial void CheckBluetoothStatus()
        {
            MainActivity.IsBluetoothOnChangedEvent += this.BluetoothChangedHandler;

            // Check if bluetooth is on.
            BluetoothManager bleManager = (BluetoothManager)MainActivity.Instance.GetSystemService(MainActivity.BluetoothService);
            if(!bleManager.Adapter.IsEnabled)
            {
                Intent enableBTIntent = new Intent(BluetoothAdapter.ActionRequestEnable);
                MainActivity.Instance.StartActivityForResult(enableBTIntent, 1);
            }
            MainActivity.Instance.Initialize();
            this.CanConnect = MainActivity.IsBluetoothOn;
        }

        /// <summary>
        /// Updates whether the phone is able to connect based on bluetooth state.
        /// </summary>
        /// <param name="sender">
        /// The object invoking the event. This should be the main activity.
        /// </param>
        /// <param name="e">
        /// Context for the event. This is ignored by this handler.
        /// </param>
        public void BluetoothChangedHandler(object sender, EventArgs e)
        {
            this.CanConnect = MainActivity.IsBluetoothOn;
        }


        private partial void StartBluetoothScan()
        {
            // Get bluetooth adapter.
            BluetoothManager bleManager = (BluetoothManager)MainActivity.Instance.GetSystemService(MainActivity.BluetoothService);
            if(bleManager != null) 
            { 
                // Get advertisement interface.
                BluetoothLeScanner scanner = bleManager.Adapter.BluetoothLeScanner;

                // Begin scanning.
                scanner.StartScan(new List<ScanFilter>()
                                  {
                                      new ScanFilter.Builder()
                                                    .SetServiceUuid(ParcelUuid.FromString("CCA85698-A7BE-4E5A-8506-9125CE3D98E8"))
                                                    .Build()
                                  },
                                  new ScanSettings.Builder()
                                                  .Build(),
                                  new GenericScanCallback()
                                  {
                                      OnScanResultCallback = (callbackType, result) => 
                                      { 
                                          this.AvailableConnections.Add(this._services.MwsFactory.Create(result.ScanRecord.DeviceName)); 
                                      },
                                      OnBatchScanResultsCallback = null,
                                      OnScanFailedCallback = null
                                  });
            }
        }


        private partial void StopBluetoothScan()
        {
            // Get bluetooth adapter.
            BluetoothManager bleManager = (BluetoothManager)MainActivity.Instance.GetSystemService(MainActivity.BluetoothService);
            if(bleManager != null)
            {
                App.Current.Dispatcher.Dispatch(() =>
                {
                    // Get advertisement interface.
                    BluetoothLeScanner scanner = bleManager.Adapter.BluetoothLeScanner;

                    // Stop scanning.
                    scanner.StopScan(new GenericScanCallback());
                });
                
            }
        }
    }
}
