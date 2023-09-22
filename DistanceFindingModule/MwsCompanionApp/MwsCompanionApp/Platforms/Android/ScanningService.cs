using Android;
using Android.Bluetooth;
using Android.Content;
using Android.Systems;
using AndroidX.Core.Content;
using MwsCompanionApp.Interfaces;
using MwsCompanionApp.Objects;
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
            this.AvailableConnections.Add(this._services.MwsFactory.Create("MWS-1"));
            this.AvailableConnections.Add(this._services.MwsFactory.Create("MWS-2"));
            this.AvailableConnections.Add(this._services.MwsFactory.Create("MWS-3"));
        }


        private partial void StopBluetoothScan() 
        { 
            
        }
    }
}
