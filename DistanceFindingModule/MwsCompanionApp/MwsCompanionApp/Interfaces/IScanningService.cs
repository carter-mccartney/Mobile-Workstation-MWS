using MwsCompanionApp.Objects;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Interfaces
{
    /// <summary>
    /// Represents a service responsible for locating MWS units.
    /// </summary>
    public interface IScanningService : INotifyPropertyChanged
    {
        /// <summary>
        /// Whether a connection is possible to make.
        /// </summary>
        bool CanConnect { get; }

        /// <summary>
        /// The list of available connections.
        /// </summary>
        ICollection<Mws> AvailableConnections { get; }

        /// <summary>
        /// Initializes the connection service.
        /// </summary>
        void Initialize();

        /// <summary>
        /// Resets the scanning service.
        /// </summary>
        void Reset();

        /// <summary>
        /// Scans for active MWS units for a given time.
        /// </summary>
        /// <param name="scanLength">
        /// The length of time to scan in seconds.
        /// </param>
        Task Scan(double scanLength);
    }
}
