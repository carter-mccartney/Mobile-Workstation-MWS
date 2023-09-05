using MauiTest1.Interfaces;
using MauiTest1.Objects;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace MauiTest1.ViewModels
{
    public class MwsConnectViewModel : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        private IMwsConnectionService _connection;

        private List<MwsConnection> _visibleConnections;
        public List<MwsConnection> VisibleConnections
        {
            get => this._visibleConnections;
            set
            {
                this.VisibleConnections?.Clear();
                this._visibleConnections = value;
                this.RaisePropertyChanged();
            }
        }
        private MwsConnection _currentConnection;
        public MwsConnection CurrentConnection 
        {
            get => this._currentConnection;
            set 
            {
                if(this._currentConnection == null ||
                   !this._currentConnection.IsConnected)
                {
                    this._currentConnection = value;
                    this.RaisePropertyChanged();
                }
            }
        }
        private bool _isRefreshing;
        public bool IsRefreshing 
        {
            get => this._isRefreshing;
            set 
            {
                this._isRefreshing = value;
                this.RaisePropertyChanged();
            }
        }
        private bool _canConnect;
        public bool CanConnect 
        {
            get => this._canConnect;
            set 
            {
                this._canConnect = value;
                this.RaisePropertyChanged();
            }
        }

        public MwsConnectViewModel(IMwsConnectionService connection) 
        {
            this._connection = connection;
            this.CanConnect = connection.CanConnect;
            this._connection.CanConnectChanged += this.CanConnectChangedHandler;
        }


        public void Initialize() 
        {
            this._connection.Initialize();
        }


        public async Task Refresh() 
        {
            if(this.CanConnect)
            {
                this.IsRefreshing = true;
                this.VisibleConnections = (await this._connection.LocateAvailableConnections()).ToList();
                this.IsRefreshing = false;
            }
        }


        public void Connect(MwsConnection connection) 
        { 
            this._connection.Connect(connection);
            connection.IsConnected = true;
        }


        public void Disconnect()
        {
            this._connection.Disconnect(this._currentConnection);
            this.CurrentConnection.IsConnected = false;
        }

        private void CanConnectChangedHandler(object sender, EventArgs e) 
        {
            this.CanConnect = this._connection.CanConnect;
            if(this._currentConnection != null &&
               this.CurrentConnection.IsConnected &&
               !this.CanConnect)
            {
                this._connection.Disconnect(this._currentConnection);
                this.CurrentConnection.IsConnected = false;
            }
            if(this.VisibleConnections != null &&
               this.VisibleConnections.Any()) 
            {
                this.VisibleConnections = new List<MwsConnection>();
            }
        }

        protected virtual void RaisePropertyChanged([CallerMemberName] string propertyName = null) 
        {
            App.Current.Dispatcher.Dispatch(() => this.PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName)));
        }
    }
}
