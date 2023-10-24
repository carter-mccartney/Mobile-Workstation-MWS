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

        private partial Task<bool> SendConnectionRequest()
        {
            return Task.Run(() => true);
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
            return true;
        }

        private partial void EndLocationSharing() 
        { 
            
        }
    }
}
