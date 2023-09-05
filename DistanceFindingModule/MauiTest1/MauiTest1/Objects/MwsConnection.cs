using MauiTest1.ViewModels;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;

namespace MauiTest1.Objects
{
    public class MwsConnection : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        private string _name;
        public string Name 
        {
            get => this._name; 
            set 
            { 
                this._name = value;
                this.RaisePropertyChanged();
            }
        }
        private byte[] _macAddress;
        public byte[] MacAddress 
        {
            get => this._macAddress;
            set 
            {
                this._macAddress = value;
                this.RaisePropertyChanged();
                this.RaisePropertyChanged(nameof(this.MacAddressString));
            }
        }

        public string MacAddressString => Regex.Replace(Convert.ToHexString(this.MacAddress).ToUpper(), ".{2}", "$0:").Trim(':');
        private bool _isConnected;
        public bool IsConnected 
        {
            get => this._isConnected;
            set 
            { 
                this._isConnected = value;
                this.RaisePropertyChanged();
            }
        }
        private double _range;
        public double Range 
        {
            get => this._range;
            set 
            {
                this._range = value;
                this.RaisePropertyChanged();
            }
        }

        protected virtual void RaisePropertyChanged([CallerMemberName] string propertyName = null)
        {
            App.Current.Dispatcher.Dispatch(() => this.PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName)));
        }
    }
}
