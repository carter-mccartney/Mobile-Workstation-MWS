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
    /// Represents a service for handling connection to the MWS.
    /// </summary>
    public interface IConnectionManagementService : INotifyPropertyChanged
    {
        /// <summary>
        /// Notifies listeners of a notification.
        /// </summary>
        event EventHandler<NotificationEventArgs> NotificationReceived;

        /// <summary>
        /// The current connection to the MWS.
        /// </summary>
        Mws CurrentConnection { get; set; }

        /// <summary>
        /// Initializes the service.
        /// </summary>
        void Initialize();

        /// <summary>
        /// Connects to the given MWS.
        /// </summary>
        /// <returns>
        /// A task representing the asynchronous state of the operation.
        /// </returns>
        Task Connect();

        /// <summary>
        /// Disconnects from the current MWS.
        /// </summary>
        void Disconnect();

        /// <summary>
        /// Begins the process of sharing the location with the connected MWS.
        /// </summary>
        /// <returns>
        /// A task representing the async state of the operation.
        /// </returns>
        Task EnterFollowerMode();

        /// <summary>
        /// Ends the process of sharing the location with the connected MWS.
        /// </summary>
        /// <returns>
        /// A task representing the async state of the operation.
        /// </returns>
        Task ExitFollowerMode();

        /// <summary>
        /// Renames the connected MWS.
        /// </summary>
        /// <param name="name">
        /// The new name to give the MWS.
        /// </param>
        /// <returns>
        /// Whether the operation was successful.
        /// </returns>
        bool Rename(string name);
    }

    /// <summary>
    /// Represents arguments for a notification event.
    /// </summary>
    public class NotificationEventArgs : EventArgs 
    { 
        /// <summary>
        /// The type of notification.
        /// </summary>
        public NotificationType Type { get; }

        /// <summary>
        /// The value for the notification, if any.
        /// </summary>
        public object Value { get; }

        /// <summary>
        /// Creates event arguments for a notification event.
        /// </summary>
        /// <param name="type">
        /// The type of notification.
        /// </param>
        /// <param name="value">
        /// The value for the notification, if any.
        /// </param>
        public NotificationEventArgs(NotificationType type, object value) 
        {
            this.Type = type;
            this.Value = value;
        }
    }

    /// <summary>
    /// The types of notifications.
    /// </summary>
    public enum NotificationType 
    { 
        Disconnected,
        BatteryLow,
        Stuck
    }
}
