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
    public partial class ConnectionManagementService : IConnectionManagementService
    {
        /// <inheritdoc/>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <inheritdoc/>
        public event EventHandler<NotificationEventArgs> NotificationReceived;

        private Mws _currentConnection;
        /// <inheritdoc/>
        public Mws CurrentConnection 
        {
            get => this._currentConnection;
            set 
            {
                this._currentConnection = value;
                this.RaisePropertyChanged();
            }
        }

        /// <inheritdoc/>
        public void Initialize()
        {
            this.CurrentConnection = null;
            this._services.EventService.BluetoothChanged += this.BluetoothChangedHandler;
            
        }

        /// <summary>
        /// Sends the connection request to the current MWS.
        /// </summary>
        /// <returns>
        /// Whether the connection was successful after a task.
        /// </returns>
        private partial Task<bool> SendConnectionRequest();

        /// <inheritdoc/>
        public async Task<bool> Connect()
        {
            if(await this.SendConnectionRequest())
            {
                this.CurrentConnection.IsConnected = true;
                return true;
            }
            else 
            {
                return false;
            }
        }

        /// <summary>
        /// Sends the disconnection request to the current MWS.
        /// </summary>
        private partial void SendDisconnectionRequest();

        /// <inheritdoc/>
        public void Disconnect()
        {
            this.SendDisconnectionRequest();
            this.CurrentConnection.IsConnected = false;
        }

        /// <summary>
        /// Sends a request to rename the MWS.
        /// </summary>
        /// <param name="name">
        /// The name to give the MWS.
        /// </param>
        /// <returns>
        /// Whether th eoperation was successful.
        /// </returns>
        private partial bool SendRenameRequest(string name);

        /// <inheritdoc/>
        public bool Rename(string name)
        {
            if(this.SendRenameRequest(name))
            {
                this.CurrentConnection.Name = name;
                return true;
            }
            else 
            {
                return false;
            }
        }

        /// <summary>
        /// Runs the actions required to place the MWS in the state where it will look for location and begins sharing location.
        /// </summary>
        /// <returns>
        /// Whether the location sharing successfully started.
        /// </returns>
        private partial bool SetupLocationSharing();

        /// <inheritdoc/>
        public bool StartSharingLocation()
        {
            return this.SetupLocationSharing();
        }

        /// <summary>
        /// Stops the process of sharing location.
        /// </summary>
        private partial void EndLocationSharing();

        /// <inheritdoc/>
        public void StopSharingLocation()
        {
            this.EndLocationSharing();
        }

        /// <summary>
        /// Invokes the event indicating a notification was received.
        /// </summary>
        /// <param name="sender">
        /// The object invoking the event. this should be the connection manager.
        /// </param>
        /// <param name="e">
        /// Context for the event. This includes the notification type and information about the notification.
        /// </param>
        private void NotificationReceivedHandler(object sender, NotificationEventArgs e) 
        {
            this.NotificationReceived?.Invoke(sender, e);
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

        /// <summary>
        /// Clears the current connection after a disconnect.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BluetoothChangedHandler(object sender, bool e) 
        {
            // Clear the current connection on disconnect.
            if(this.CurrentConnection != null)
            {
                this.CurrentConnection.IsConnected = false;
                this.CurrentConnection = null;
            }
        }
    }
}
