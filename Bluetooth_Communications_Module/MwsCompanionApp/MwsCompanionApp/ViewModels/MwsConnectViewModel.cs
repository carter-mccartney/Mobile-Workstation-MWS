using MwsCompanionApp.Interfaces;
using MwsCompanionApp.Objects;
using MwsCompanionApp.Services;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.ViewModels
{
    /// <summary>
    /// Represents a view-model for managing the entire application view.
    /// </summary>
    public class MwsConnectViewModel : INotifyPropertyChanged
    {
        /// <inheritdoc/>
        public event PropertyChangedEventHandler PropertyChanged;

        private ServicesContainer _services;
        /// <summary>
        /// The services for the application.
        /// </summary>
        public ServicesContainer Services 
        {
            get => this._services;
            set 
            {
                this._services = value;
                this.RaisePropertyChanged();
            }
        }

        private bool _isRefreshing;
        /// <summary>
        /// Whether a refresh is occurring.
        /// </summary>
        public bool IsRefreshing
        {
            get => this._isRefreshing;
            set
            {
                this._isRefreshing = value;
                this.RaisePropertyChanged();
            }
        }

        /// <summary>
        /// Creates a view-model for finding connections to an MWS unit.
        /// </summary>
        /// <param name="services">
        /// The services for the application.
        /// </param>
        public MwsConnectViewModel(ServicesContainer services)
        {
            this.Services = services;
        }

        /// <summary>
        /// Refreshes the set of nearby MWS units.
        /// </summary>
        public async Task Refresh()
        {
            if(this.Services.Permissions.IsBluetoothPermitted &&
               this.Services.ScanningService.CanConnect)
            {
                await this.Services.ScanningService.Scan(5);
                App.Current.Dispatcher.Dispatch(() =>
                {
                    this.IsRefreshing = false;
                });
            }
            else 
            {
                this.IsRefreshing = false;
            }
        }

        /// <summary>
        /// Raises the property changed event in a streamlined manner.
        /// </summary>
        /// <param name="propertyName">
        /// The name of the property that has changed. This defaults to the clling member's name.
        /// </param>
        protected virtual void RaisePropertyChanged([CallerMemberName] string propertyName = null)
        {
            App.Current.Dispatcher.Dispatch(() => this.PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName)));
        }
    }
}
