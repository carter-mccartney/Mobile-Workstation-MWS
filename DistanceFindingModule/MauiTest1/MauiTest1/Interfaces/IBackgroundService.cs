using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MauiTest1.Interfaces
{
    public interface IBackgroundService
    {
        bool IsBluetoothOn { get; }
        event EventHandler BluetoothChanged;
        void Initialize();
        void Start();
        void Stop();
    }
}
