using Android.Net.Wifi.Aware;
using MwsCompanionApp.Interfaces;
using MwsCompanionApp.Services;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Objects
{
    /// <summary>
    /// Represents an MWS.
    /// </summary>
    public class Mws : INotifyPropertyChanged
    {
        /// <inheritdoc/>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// The services for the application.
        /// </summary>
        public readonly ServicesContainer Services;

        private string _name;
        /// <summary>
        /// The name of the MWS.
        /// </summary>
        public string Name 
        { 
            get => this._name;
            set 
            { 
                this._name = value;
                this.RaisePropertyChanged();
            } 
        }

        private bool _canConnect;
        /// <summary>
        /// Whether a connection can be attempted.
        /// </summary>
        public bool CanConnect 
        {
            get => this._canConnect;
            set 
            {
                this._canConnect = value;
                this.RaisePropertyChanged();
            }
        }

        private bool _isConnected;
        /// <summary>
        /// Whether the MWS is connected.
        /// </summary>
        public bool IsConnected 
        { 
            get => this._isConnected;
            set 
            { 
                this._isConnected = value;
                this.RaisePropertyChanged();
            } 
        }

        /// <summary>
        /// Whether the MWS is currently doing anything preventing it from disconnecting.
        /// </summary>
        public bool IsDoingAnything => this.IsFollowing ||
                                       this.IsCalibrating;

        private bool _isFollowing;
        /// <summary>
        /// Whether the MWS is currently in follower mode.
        /// </summary>
        public bool IsFollowing 
        {
            get => this._isFollowing;
            set 
            {
                this._isFollowing = value;
                this.RaisePropertyChanged();
                this.RaisePropertyChanged(nameof(this.IsDoingAnything));
            }
        }

        private bool _isCalibrating;
        /// <summary>
        /// Whether the MWS is calibrating.
        /// </summary>
        public bool IsCalibrating 
        {
            get => this._isCalibrating;
            set 
            {
                this._isCalibrating = value;
                this.RaisePropertyChanged();
                this.RaisePropertyChanged(nameof(this.IsDoingAnything));
            }
        }

        private byte _range;
        /// <summary>
        /// The range to follow.
        /// </summary>
        public byte Range 
        {
            get => this._range;
            set 
            {
                this._range = value;
                this.RaisePropertyChanged();
            }
        }

        /// <summary>
        /// Creates a new instance of an MWS.
        /// </summary>
        /// <param name="services">
        /// The services for the application.
        /// </param>
        /// <param name="name">
        /// the name of the MWS.
        /// </param>
        public Mws(ServicesContainer services, string name) 
        { 
            this.Services = services;
            this.Name = name;
            this.IsConnected = false;
            this.CanConnect = true;
            this.IsFollowing = false;
            this.Range = 2;
        }

        /// <summary>
        /// Connects to the MWS.
        /// </summary>
        public void Connect()
        {
            this.Range = (byte)int.Parse(Preferences.Default.Get("Range", "2"));
            this.CanConnect = false;
            this.Services.ConnectionService.Connect();
        }

        /// <summary>
        /// Disconnects this MWS.
        /// </summary>
        public void Disconnect() 
        {
            this.Services.ConnectionService.Disconnect();
        }

        /// <summary>
        /// Places the MWS into follower mode.
        /// </summary>
        public void BeginFollowing()
        {
            this.Services.ConnectionService.EnterFollowerMode();
        }

        /// <summary>
        /// Stops the MWS from following.
        /// </summary>
        public void StopFollowing() 
        {
            this.Services.ConnectionService.ExitFollowerMode();
        }

        /// <summary>
        /// Calibrates target One.
        /// </summary>
        public void Calibrate1()
        {
            this.IsCalibrating = true;
            this.Services.ConnectionService.Calibrate(1);
        }

        /// <summary>
        /// Calibrates target Two.
        /// </summary>
        public void Calibrate2()
        {
            this.IsCalibrating = true;
            this.Services.ConnectionService.Calibrate(2);
        }

        /// <summary>
        /// Calibrates target Three.
        /// </summary>
        public void Calibrate3()
        {
            this.IsCalibrating = true;
            this.Services.ConnectionService.Calibrate(3);
        }

        /// <summary>
        /// Calibrates target Four.
        /// </summary>
        public void Calibrate4()
        {
            this.IsCalibrating = true;
            this.Services.ConnectionService.Calibrate(4);
        }


        public void UpdateRange()
        {
            // Set this as preferred range.
            App.Current.Dispatcher.Dispatch(() => Preferences.Default.Set("Range", this.Range.ToString()));

            this.Services.ConnectionService.UpdateRange();
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

        /// <inheritdoc/>
        public override bool Equals(object obj)
        {
            Mws that = (Mws)obj;
            return that != null && 
                   this.Name.Equals(that.Name);
        }

        /// <inheritdoc/>
        public override int GetHashCode()
        {
            return this.Name.GetHashCode();
        }
    }
}
