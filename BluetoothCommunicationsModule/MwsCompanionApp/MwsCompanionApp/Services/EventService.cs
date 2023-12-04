using MwsCompanionApp.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Services
{
    /// <inheritdoc/>
    public class EventService : IEventService
    {
        private event EventHandler _resumed;
        /// <inheritdoc/>
        public event EventHandler Resumed
        {
            add 
            {
                this._resumed += value;
                this._resumedHandlers.Add(value);
            }
            remove
            {
                this._resumed -= value;
                this._resumedHandlers.Remove(value);
            }
        }
        /// <summary>
        /// The list of event handlers for the resumed event.
        /// </summary>
        private List<EventHandler> _resumedHandlers;

        private event EventHandler _sleepStarted;
        /// <inheritdoc/>
        public event EventHandler SleepStarted
        {
            add
            {
                this._sleepStarted += value;
                this._sleepStartedHandlers.Add(value);
            }
            remove
            {
                this._sleepStarted -= value;
                this._sleepStartedHandlers.Remove(value);
            }
        }
        /// <summary>
        /// The list of event handlers for the sleep started event.
        /// </summary>
        private List<EventHandler> _sleepStartedHandlers;

        private event EventHandler _appClosed;
        /// <inheritdoc/>
        public event EventHandler AppClosed
        {
            add
            {
                this._appClosed += value;
                this._appClosedHandlers.Add(value);
            }
            remove
            {
                this._appClosed -= value;
                this._appClosedHandlers.Remove(value);
            }
        }
        /// <summary>
        /// The list of event handlers for the app closed event.
        /// </summary>
        private List<EventHandler> _appClosedHandlers;

        private event EventHandler<bool> _bluetoothChanged;
        /// <inheritdoc/>
        public event EventHandler<bool> BluetoothChanged
        {
            add
            {
                this._bluetoothChanged += value;
                this._bluetoothChangedHandlers.Add(value);
            }
            remove
            {
                this._bluetoothChanged -= value;
                this._bluetoothChangedHandlers.Remove(value);
            }
        }
        /// <summary>
        /// The list of event handlers for the bluetooth changed event.
        /// </summary>
        private List<EventHandler<bool>> _bluetoothChangedHandlers;

        private event EventHandler<UIMessageEventArgs> _uiMessageDispatched;
        /// <inheritdoc/>
        public event EventHandler<UIMessageEventArgs> UIMessageDispatched
        {
            add
            {
                this._uiMessageDispatched += value;
                this._uiMessageDispatchedHandlers.Add(value);
            }
            remove
            {
                this._uiMessageDispatched -= value;
                this._uiMessageDispatchedHandlers.Remove(value);
            }
        }
        /// <summary>
        /// The list of event handlers for the UI message dispatched event.
        /// </summary>
        private List<EventHandler<UIMessageEventArgs>> _uiMessageDispatchedHandlers;

        private event EventHandler<SystemNotificationEventArgs> _systemNotificationDispatched;
        /// <inheritdoc/>
        public event EventHandler<SystemNotificationEventArgs> SystemNotificationDispatched
        {
            add
            {
                this._systemNotificationDispatched += value;
                this._systemNotificationDispatchedHandlers.Add(value);
            }
            remove
            {
                this._systemNotificationDispatched -= value;
                this._systemNotificationDispatchedHandlers.Remove(value);
            }
        }
        /// <summary>
        /// The list of event handlers for the system notification dispatched event.
        /// </summary>
        private List<EventHandler<SystemNotificationEventArgs>> _systemNotificationDispatchedHandlers;

        /// <summary>
        /// Creates the event service.
        /// </summary>
        public EventService() 
        {
            this._resumedHandlers = new List<EventHandler>();
            this._sleepStartedHandlers = new List<EventHandler>();
            this._appClosedHandlers = new List<EventHandler>();
            this._bluetoothChangedHandlers = new List<EventHandler<bool>>();
            this._uiMessageDispatchedHandlers = new List<EventHandler<UIMessageEventArgs>>();
            this._systemNotificationDispatchedHandlers = new List<EventHandler<SystemNotificationEventArgs>>();
        }

        /// <inheritdoc/>
        public void InvokeResumedEvent(object sender, EventArgs e)
        {
            this._resumed?.Invoke(sender, e);
        }

        /// <inheritdoc/>
        public void InvokeSleepStarted(object sender, EventArgs e)
        {
            this._sleepStarted?.Invoke(sender, e);
        }

        /// <inheritdoc/>
        public void InvokeAppClosedEvent(object sender, EventArgs e)
        {
            this._appClosed?.Invoke(sender, e);
            foreach(EventHandler handler in this._resumedHandlers) 
            {
                this._resumed -= handler;
            }
            foreach(EventHandler handler in this._sleepStartedHandlers)
            {
                this._sleepStarted -= handler;
            }
            foreach(EventHandler handler in this._appClosedHandlers)
            {
                this._appClosed -= handler;
            }
            foreach(EventHandler<bool> handler in this._bluetoothChangedHandlers) 
            {
                this._bluetoothChanged -= handler;
            }
            foreach(EventHandler<UIMessageEventArgs> handler in this._uiMessageDispatchedHandlers) 
            {
                this._uiMessageDispatched -= handler;
            }
            foreach(EventHandler<SystemNotificationEventArgs> handler in this._systemNotificationDispatchedHandlers) 
            { 
                this._systemNotificationDispatched -= handler;
            }
        }

        /// <inheritdoc/>
        public void InvokeBluetoothChangedEvent(object sender, bool e) 
        { 
            this._bluetoothChanged?.Invoke(sender, e);
        }

        /// <inheritdoc/>
        public void InvokeUIMessageDispatchedEvent(object sender, UIMessageEventArgs e)
        {
            this._uiMessageDispatched?.Invoke(sender, e);
        }

        /// <inheritdoc/>
        public void InvokeSystemNotificationDispatchedEvent(object sender, SystemNotificationEventArgs e)
        {
            this._systemNotificationDispatched?.Invoke(sender, e);
        }
    }
}
