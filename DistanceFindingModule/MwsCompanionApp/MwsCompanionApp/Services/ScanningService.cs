using MwsCompanionApp.Interfaces;
using MwsCompanionApp.Objects;
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
    public partial class ScanningService : IScanningService
    {
        /// <inheritdoc/>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// The factory for MWS instances.
        /// </summary>
        private ServicesContainer _services;

        private bool _canConnect;
        /// <inheritdoc/>
        public bool CanConnect
        {
            get => this._canConnect;
            set
            {
                this._canConnect = value;
                this.RaisePropertyChanged();

                // Reset everything if this is false.
                this._services.EventService.InvokeBluetoothChangedEvent(this, value);
                if(!value) 
                { 
                    this.AvailableConnections.Clear();
                    this._services.EventService.InvokeBluetoothChangedEvent(this, false);
                }
            }
        }

        private HashSet<Mws> _availableConnections;
        /// <inheritdoc/>
        public ICollection<Mws> AvailableConnections
        {
            get => this._availableConnections;
            private set
            {
                // Cast is safe since set is only called in class.
                this._availableConnections = (HashSet<Mws>)value;
                this.RaisePropertyChanged();
            }
        }

        /// <summary>
        /// Checks for the state of bluetooth.
        /// </summary>
        private partial void CheckBluetoothStatus();

        /// <inheritdoc/>
        public void Initialize()
        {
            this.CheckBluetoothStatus();
        }

        /// <summary>
        /// Starts scanning for MWS units over bluetooth.
        /// </summary>
        private partial void StartBluetoothScan();

        /// <summary>
        /// Stops scanning for MWS units over bluetooth.
        /// </summary>
        private partial void StopBluetoothScan();

        /// <inheritdoc/>
        public async Task Scan(double scanLength)
        {
            this.AvailableConnections.Clear();

            // Begin scanning.
            this.StartBluetoothScan();

            // Wait for the length of the scan time.
            await Task.Delay(TimeSpan.FromSeconds(scanLength));

            // Stop scanning.
            this.StopBluetoothScan();

            // Update the UI.
            this.RaisePropertyChanged(nameof(this.AvailableConnections));
        }

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
