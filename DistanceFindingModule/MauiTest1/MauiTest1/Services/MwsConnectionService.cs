using MauiTest1.Interfaces;
using MauiTest1.Objects;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MauiTest1.Services
{
    public class MwsConnectionService : IMwsConnectionService
    {
        private bool _canConnect;
        public bool CanConnect 
        {
            get => this._canConnect;
            private set 
            {
                this._canConnect = value;
                this.CanConnectChanged?.Invoke(this, new EventArgs());
            }
        }

        public event EventHandler CanConnectChanged;

        private IBackgroundService _backgroundService;

        int Index = 0;

        public MwsConnectionService(IBackgroundService backgroundService)
        {
            this._backgroundService = backgroundService;
            this._backgroundService.BluetoothChanged += this.BluetoothChangedHandler;
            this.CanConnect = this._backgroundService.IsBluetoothOn;
        }

        public void Initialize() 
        {
            this._backgroundService.Initialize();
        }

        public void Connect(MwsConnection connection)
        {
            this._backgroundService.Start();
        }

        public void Disconnect(MwsConnection connection)
        {
            this._backgroundService.Stop();
        }

        public async Task<IEnumerable<MwsConnection>> LocateAvailableConnections()
        {
            await Task.Delay(500);
            List<MwsConnection> result = new List<MwsConnection>();
            if(this.Index % 2 == 0) 
            {
                result.Add(new MwsConnection() 
                { 
                    MacAddress = new byte[6] 
                    { 
                        (byte)1, (byte)1, (byte)1, (byte)1, (byte)1, (byte)1
                    },
                    IsConnected = false,
                    Name = "MWS 1",
                    Range = 10
                });
                result.Add(new MwsConnection()
                {
                    MacAddress = new byte[6]
                    {
                        (byte)10, (byte)12, (byte)68, (byte)120, (byte)12, (byte)2
                    },
                    IsConnected = false,
                    Name = "MWS 2",
                    Range = 10
                });
                result.Add(new MwsConnection()
                {
                    MacAddress = new byte[6]
                    {
                        (byte)255, (byte)255, (byte)75, (byte)23, (byte)145, (byte)1
                    },
                    IsConnected = false,
                    Name = "MWS 3",
                    Range = 10
                });
            }
            this.Index++;
            return result;
        }

        private void BluetoothChangedHandler(object sender, EventArgs e) 
        {
            this.CanConnect = this._backgroundService.IsBluetoothOn;
        }
    }
}
