using MwsCompanionApp.Interfaces;
using MwsCompanionApp.Objects;
using System;
using System.Collections.Generic;
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
