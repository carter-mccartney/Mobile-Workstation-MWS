using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Interfaces
{
    /// <summary>
    /// Represents a service for aggregating events for usbscribing and publishing.
    /// </summary>
    public interface IEventService
    {
        /// <summary>
        /// The event indicating the application has resumed.
        /// </summary>
        event EventHandler Resumed;

        /// <summary>
        /// The event indicating the application has began to sleep.
        /// </summary>
        event EventHandler SleepStarted;

        /// <summary>
        /// The event indicating the application has been closed.
        /// </summary>
        event EventHandler AppClosed;

        /// <summary>
        /// The event indicating that access to bluetooth has been changed.
        /// </summary>
        event EventHandler<bool> BluetoothChanged;

        /// <summary>
        /// The event indicating that a message intended for the user over the UI has been generated.
        /// </summary>
        event EventHandler<UIMessageEventArgs> UIMessageDispatched;

        /// <summary>
        /// The event indicating that a notification intended for display in the system UI has been generated.
        /// </summary>
        event EventHandler<SystemNotificationEventArgs> SystemNotificationDispatched;

        /// <summary>
        /// Invokes the resumed event.
        /// </summary>
        /// <param name="sender">
        /// The object that invoked the event.
        /// </param>
        /// <param name="e">
        /// Context for the event.
        /// </param>
        void InvokeResumedEvent(object sender, EventArgs e);

        /// <summary>
        /// Invokes the sleep started event.
        /// </summary>
        /// <param name="sender">
        /// The object invoking the event.
        /// </param>
        /// <param name="e">
        /// Context for the event.
        /// </param>
        void InvokeSleepStarted(object sender, EventArgs e);

        /// <summary>
        /// Invokes the app closed event.
        /// </summary>
        /// <param name="sender">
        /// The object invoking the event.
        /// </param>
        /// <param name="e">
        /// Context for the event.
        /// </param>
        void InvokeAppClosedEvent(object sender, EventArgs e);

        /// <summary>
        /// Invokes the bluetooth changed event.
        /// </summary>
        /// <param name="sender">
        /// The object invoking the event.
        /// </param>
        /// <param name="e">
        /// Context for the event. True means bluetooth has reconnected and false 
        /// means it has disconnected.
        /// </param>
        void InvokeBluetoothChangedEvent(object sender, bool e);

        /// <summary>
        /// Invokes the UI message dispatched event.
        /// </summary>
        /// <param name="sender">
        /// The object invoking the event.
        /// </param>
        /// <param name="e">
        /// Context for the event, including the message length and duration.
        /// </param>
        void InvokeUIMessageDispatchedEvent(object sender, UIMessageEventArgs e);

        /// <summary>
        /// Invokes the system notification dispatched event.
        /// </summary>
        /// <param name="sender">
        /// The object invoking the event.
        /// </param>
        /// <param name="e">
        /// Context for the event, including the title, subtitle, description, category, icon path, 
        /// channel name, and whether it is urgent.
        /// </param>
        void InvokeSystemNotificationDispatchedEvent(object sender, SystemNotificationEventArgs e);
    }

    /// <summary>
    /// Represents arguments for a notification event.
    /// </summary>
    public class UIMessageEventArgs : EventArgs
    {
        /// <summary>
        /// The message to display.
        /// </summary>
        public string Message { get; private set; }

        /// <summary>
        /// Whether the message is long.
        /// </summary>
        public bool IsLong { get; private set; }

        /// <summary>
        /// Creates event arguments for a UI message event.
        /// </summary>
        /// <param name="message">
        /// The text of the message.
        /// </param>
        /// <param name="isLong">
        /// Whether the message is long. This is false by default.
        /// </param>
        public UIMessageEventArgs(string message, bool isLong = false)
        {
            this.Message = message;
            this.IsLong = isLong;
        }
    }

    /// <summary>
    /// Represents arguments for a system notification event.
    /// </summary>
    public class SystemNotificationEventArgs : EventArgs 
    { 
        /// <summary>
        /// The title of the notification.
        /// </summary>
        public string Title { get; private set; }

        /// <summary>
        /// The subtitle of the notification.
        /// </summary>
        public string Subtitle { get; private set; }

        /// <summary>
        /// The description of the notification.
        /// </summary>
        public string Description { get; private set; }

        /// <summary>
        /// The name of the channel for the notification.
        /// </summary>
        public string Channel { get; private set; }

        /// <summary>
        /// The path to the icon image for the notification
        /// </summary>
        public string IconPath { get; private set; }

        /// <summary>
        /// The category of notification.
        /// </summary>
        public NotificationCategory Category { get; private set; }

        /// <summary>
        /// Whether the notification must be seen immediately.
        /// </summary>
        public bool IsUrgent { get; private set; }

        /// <summary>
        /// Creates a new system notification.
        /// </summary>
        /// <param name="title">
        /// The title of the notification.
        /// </param>
        /// <param name="description">
        /// A description of the notification.
        /// </param>
        /// <param name="subtitle">
        /// The subtitle of the notification. This defaults to null.
        /// </param>
        /// <param name="channel">
        /// The channel of the notification. This defaults to null.
        /// </param>
        /// <param name="iconPath">
        /// The path to the notification icon. This defaults to null.
        /// </param>
        /// <param name="category">
        /// The category of the notification. This defaults to unspecified.
        /// </param>
        /// <param name="isUrgent">
        /// Whether the notification is urgent. This defaults to false.
        /// </param>
        public SystemNotificationEventArgs(string title, 
                                           string description, 
                                           string subtitle = null, 
                                           string channel = null, 
                                           string iconPath = null, 
                                           NotificationCategory category = NotificationCategory.Unspecified, 
                                           bool isUrgent = false) 
        { 
            this.Title = title;
            this.Description = description;
            this.Subtitle = subtitle;
            this.Channel = channel;
            this.IconPath = iconPath;
            this.Category = category;
            this.IsUrgent = isUrgent;
        }
    }

    /// <summary>
    /// The categories of notifications.
    /// </summary>
    public enum NotificationCategory 
    { 
        /// <summary>
        /// Represents an unspecified category of notification.
        /// </summary>
        Unspecified,
        /// <summary>
        /// Represents a status update.
        /// </summary>
        Status,
        /// <summary>
        /// Represents an error notification.
        /// </summary>
        Error,
        /// <summary>
        /// Represents a notification about the progress of a long-running task.
        /// </summary>
        Progress,
        /// <summary>
        /// Represents a notification that a background service is running.
        /// </summary>
        Service
    }
}
