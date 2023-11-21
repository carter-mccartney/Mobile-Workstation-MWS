using MwsCompanionApp.Interfaces;
using Plugin.LocalNotification;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Services
{
    /// <inheritdoc/>
    public partial class PermissionsService : IPermissionsService
    {
        /// <inheritdoc/>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// The container for all application services.
        /// </summary>
        private ServicesContainer _services;

        private bool _isBluetoothPermitted;
        /// <inheritdoc/>
        public bool IsBluetoothPermitted 
        {
            get => this._isBluetoothPermitted;
            private set 
            {
                this._isBluetoothPermitted = value;
                this.RaisePropertyChanged();
            }
        }

        /// <summary>
        /// Creates a service for permissions.
        /// </summary>
        /// <param name="services">
        /// The application services.
        /// </param>
        public PermissionsService(ServicesContainer services) 
        {
            this._services = services;
            this.IsBluetoothPermitted = false;
        }

        /// <inheritdoc/>
        public async Task CheckPermissions() 
        {
            // Get system-specific permissions.
            App.Current.Dispatcher.Dispatch(this.GetSystemPermissions);
        }

        /// <summary>
        /// Gets the OS specific permissions.
        /// </summary>
        public partial void GetSystemPermissions();

        /// <summary>
        /// Raises the property changed event in a streamlined manner.
        /// </summary>
        /// <param name="propertyName">
        /// The name of the property that has changed. This defaults to the clling member's name.
        /// </param>
        protected void RaisePropertyChanged([CallerMemberName] string propertyName = null)
        {
            App.Current.Dispatcher.Dispatch(() => this.PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName)));
        }
    }
}
