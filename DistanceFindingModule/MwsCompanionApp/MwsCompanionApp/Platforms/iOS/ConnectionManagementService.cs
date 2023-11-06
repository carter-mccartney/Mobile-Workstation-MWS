using MwsCompanionApp.Objects;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Services
{
    public partial class ConnectionManagementService
    {
        /// <summary>
        /// The services avaiable.
        /// </summary>
        private ServicesContainer _services;

        private partial bool IsConnected() 
        {
            return false;
        }

        /// <summary>
        /// Creates a connection management service with the given services.
        /// </summary>
        /// <param name="services">
        /// The services available to this service.
        /// </param>
        public ConnectionManagementService(ServicesContainer services)
        {
            this._services = services;
            this._isConnectionSetUp = false;
            this._hasManufacturerBeenConfirmed = false;
            this._hasModelBeenConfirmed = false;
        }

        private partial Task SendConnectionRequest()
        {
            return Task.CompletedTask;
        }

        private partial void SendDisconnectionRequest()
        {

        }

        private partial void ReadValue(string serviceUuid, string characteristicUuid)
        {
            
        }

        private partial Task WriteValue(string serviceUuid, string characteristicUuid, byte[] value) 
        {
            return Task.CompletedTask;
        }

        private partial bool SendRenameRequest(string name)
        {
            return true;
        }

        private partial byte[] GenerateSignature() 
        {
            return null;
        }

        private partial bool StartAdvertisingLocation() 
        {
            return true;
        }

        private partial void StopAdvertisingLocation() 
        { 
            
        }
    }
}
