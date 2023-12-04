using MwsCompanionApp.Interfaces;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Services
{
    /// <summary>
    /// Represents a container for services.
    /// </summary>
    public class ServicesContainer : INotifyPropertyChanged
    {
        /// <inheritdoc/>
        public event PropertyChangedEventHandler PropertyChanged;

        private readonly Lazy<IEventService> _eventService;
        /// <summary>
        /// The event service for the application.
        /// </summary>
        public IEventService EventService => this._eventService.Value;

        private readonly Lazy<IConnectionManagementService> _connectionService;
        /// <summary>
        /// The MWS connection management service for the application.
        /// </summary>
        public IConnectionManagementService ConnectionService => this._connectionService.Value;

        private readonly Lazy<IScanningService> _scanningService;
        /// <summary>
        /// The MWS connection scanning service for the application.
        /// </summary>
        public IScanningService ScanningService => this._scanningService.Value;

        private readonly Lazy<IMwsFactory> _mwsFactory;
        /// <summary>
        /// The MWS factory for the application.
        /// </summary>
        public IMwsFactory MwsFactory => this._mwsFactory.Value;

        private readonly Lazy<IPermissionsService> _permissions;
        /// <summary>
        /// The permissions service for the application.
        /// </summary>
        public IPermissionsService Permissions => this._permissions.Value;

        /// <summary>
        /// Creates a service container and injects the services into it.
        /// </summary>
        /// <param name="eventService">
        /// The event service for the application.
        /// </param>
        /// <param name="connectionService">
        /// The MWS connection management service for the application.
        /// </param>
        /// <param name="scanningService">
        /// The MWS connection scanning service for the application.
        /// </param>
        /// <param name="mwsFactory">
        /// The MWS factory for the application.
        /// </param>
        /// <param name="permissions">
        /// The permissions service.
        /// </param>
        public ServicesContainer(Lazy<IEventService> eventService, 
                                 Lazy<IConnectionManagementService> connectionService, 
                                 Lazy<IScanningService> scanningService, 
                                 Lazy<IMwsFactory> mwsFactory,
                                 Lazy<IPermissionsService> permissions)
        { 
            this._eventService = eventService;
            this._connectionService = connectionService;
            this._scanningService = scanningService;
            this._mwsFactory = mwsFactory;
            this._permissions = permissions;

            // Reinitialize services on resume.
            this.EventService.Resumed += async (s, e) =>
            {
                await this.Permissions.CheckPermissions();
                this.ScanningService.Initialize();
            };
        }

        /// <summary>
        /// Initializes all of the services.
        /// </summary>
        public async Task InitializeServices()
        {
            await this.Permissions.CheckPermissions();
            while(!this.Permissions.IsBluetoothPermitted) await Task.Delay(TimeSpan.FromSeconds(1));
            this.ScanningService.Initialize();
            this.ConnectionService.Initialize();
            IMwsFactory factory = this.MwsFactory; // Make sure it exists now.
            IEventService eventService = this.EventService;
        }
    }
}
