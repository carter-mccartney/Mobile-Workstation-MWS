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
}
