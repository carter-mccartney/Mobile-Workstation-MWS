using MauiTest1.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MauiTest1.Services
{
    public partial class BackgroundService : IBackgroundService
    {
        private bool _isBluetoothOn;
        public bool IsBluetoothOn
        {
            get => this._isBluetoothOn;
            set
            {
                this._isBluetoothOn = value;
                this.BluetoothChanged?.Invoke(this, new EventArgs());
            }
        }
        public event EventHandler BluetoothChanged;
        public partial void Initialize();
        public partial void Start();

        public partial void Stop();
    }
}
