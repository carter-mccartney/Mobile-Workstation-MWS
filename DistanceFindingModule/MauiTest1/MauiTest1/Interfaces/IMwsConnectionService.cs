using MauiTest1.Objects;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MauiTest1.Interfaces
{
    public interface IMwsConnectionService
    {
        bool CanConnect { get; }
        event EventHandler CanConnectChanged;
        void Initialize();
        Task<IEnumerable<MwsConnection>> LocateAvailableConnections();
        void Connect(MwsConnection connection);
        void Disconnect(MwsConnection connection);
    }
}
