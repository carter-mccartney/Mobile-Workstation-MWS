using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Interfaces
{
    /// <summary>
    /// Represents a service for requesting permissions.
    /// </summary>
    public interface IPermissionsService : INotifyPropertyChanged
    {
        /// <summary>
        /// Whether bluetooth permissions are enabled.
        /// </summary>
        bool IsBluetoothPermitted { get; }

        /// <summary>
        /// Checks for all required permissions and requests them if necessary.
        /// </summary>
        void CheckPermissions();
    }
}
