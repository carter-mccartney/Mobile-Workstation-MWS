using MwsCompanionApp.Interfaces;
using MwsCompanionApp.Objects;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Services
{
    /// <inheritdoc/>
    public partial class ConnectionManagementService : IConnectionManagementService
    {
        /// <inheritdoc/>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <inheritdoc/>
        public event EventHandler<NotificationEventArgs> NotificationReceived;

        private Mws _currentConnection;
        /// <inheritdoc/>
        public Mws CurrentConnection 
        {
            get => this._currentConnection;
            set 
            {
                this._currentConnection = value;
                this.RaisePropertyChanged();
            }
        }

        /// <summary>
        /// Whether the device is connected.
        /// </summary>
        private partial bool IsConnected();

        /// <summary>
        /// Whether a connection is set up.
        /// </summary>
        private bool _isConnectionSetUp;

        /// <summary>
        /// Whether the manufacturer has been confirmed to be correct.
        /// </summary>
        private bool _hasManufacturerBeenConfirmed;

        /// <summary>
        /// Whether the model has been confirmed to be correct.
        /// </summary>
        private bool _hasModelBeenConfirmed;

        /// <inheritdoc/>
        public void Initialize()
        {
            this.CurrentConnection = null;
            this._services.EventService.BluetoothChanged += this.BluetoothChangedHandler;
        }

        /// <summary>
        /// Sends the connection request to the current MWS.
        /// </summary>
        /// <returns>
        /// A task representing the asynchronous state of the operation.
        /// </returns>
        private partial Task SendConnectionRequest();

        /// <inheritdoc/>
        public async Task Connect()
        {
            // Used to coordinate in messaging the user.
            object lockObject = new object();

            // Display a connection message periodically.
            this._services.EventService.InvokeUIMessageDispatchedEvent(this, new UIMessageEventArgs("Connecting..."));
            _ = Task.Run(async () => // Ignore the execution of the task.
            {
                // This is good enough for a low-effort loading animation.
                uint dotCount = 0;
                while(!this.CurrentConnection.CanConnect)
                {
                    await Task.Delay(TimeSpan.FromSeconds(1)); // 1 second seems like a good refresh rate.
                    lock(lockObject) // Don't display if done. Syncronize with finishing up.
                    {
                        if(!this.CurrentConnection.CanConnect) 
                        {
                            // Build message.
                            byte currentCount = (byte)(dotCount & 0x3); // Only care about the value mod 4. Fast computation of mod 4.
                            string message = "Connecting" + (currentCount == 1 ? "." : currentCount == 2 ? ".." : currentCount == 3 ? "..." : "");
                            App.Current.Dispatcher.Dispatch(() => this._services.EventService.InvokeUIMessageDispatchedEvent(this, new UIMessageEventArgs(message)));
                        }
                    }
                    dotCount++;
                }
            });

            // Send the request to connect.
            await this.SendConnectionRequest();

            // Check if the initial connection went through.
            if(this.IsConnected() &&
               (await this.SetupAndConfirmConnection())) // Ensure that setup confirms the proper server specification.
            {
                // The connection succeeded.
                await Task.Run(() =>
                {
                    lock(lockObject)
                    {
                        App.Current.Dispatcher.Dispatch(() =>
                        {
                            this.CurrentConnection.IsConnected = true;
                            this.CurrentConnection.CanConnect = true;

                            // Check if calibration has been done and suggest it if not.
                            string appData = FileSystem.Current.AppDataDirectory;
                            try
                            {
                                string filePath = Path.Combine(appData, this.CurrentConnection.Name);
                                if(!File.Exists(filePath))
                                {
                                    this._services.EventService.InvokeUIMessageDispatchedEvent(this, new UIMessageEventArgs("Calibration is recommended before use.", true));
                                }
                                else
                                {
                                    // Load the calibration stored.
                                    string fileContents = File.ReadAllText(filePath);
                                    string[] lines = fileContents.Split('\n');
                                    int rssi1 = int.Parse(lines[0]);
                                    int rssi2 = int.Parse(lines[1]);
                                    int rssi3 = int.Parse(lines[2]);
                                    int rssi4 = int.Parse(lines[3]);

                                    Task.Run(async () =>
                                    {
                                        await this.LoadCalibrationValues(rssi1, rssi2, rssi3, rssi4);
                                    });
                                }
                            }
                            catch(Exception) { } // Do nothing.
                        });
                    }
                });
            }
            else
            {
                // The connection failed one way or another.
                await Task.Run(() =>
                {
                    lock(lockObject)
                    {
                        App.Current.Dispatcher.Dispatch(() =>
                        {
                            this._services.EventService.InvokeUIMessageDispatchedEvent(this, new UIMessageEventArgs("Connection Failed."));
                            if(this.CurrentConnection != null)
                            {
                                this.CurrentConnection.CanConnect = true;
                            }
                        });
                    }
                });
            }
        }

        /// <inheritdoc/>
        public async Task LoadCalibrationValues(int rssi1, int rssi2, int rssi3, int rssi4) 
        {
            // Write to the One, Two, Three, and Four characteristics.
            await this.WriteValue(MwsUuidStrings.CalibrationServiceUuid, MwsUuidStrings.CalibrationOneUuid, BitConverter.GetBytes(rssi1));
            await this.WriteValue(MwsUuidStrings.CalibrationServiceUuid, MwsUuidStrings.CalibrationTwoUuid, BitConverter.GetBytes(rssi2));
            await this.WriteValue(MwsUuidStrings.CalibrationServiceUuid, MwsUuidStrings.CalibrationThreeUuid, BitConverter.GetBytes(rssi3));
            await this.WriteValue(MwsUuidStrings.CalibrationServiceUuid, MwsUuidStrings.CalibrationFourUuid, BitConverter.GetBytes(rssi4));
        }


        private async Task<bool> SetupAndConfirmConnection()
        {
            // Wait for connection to be set up.
            while(this.IsConnected() &&
                  (!this._isConnectionSetUp ||
                  !this._hasManufacturerBeenConfirmed ||
                  !this._hasModelBeenConfirmed))
            {
                await Task.Delay(TimeSpan.FromSeconds(5));
            }

            return this._isConnectionSetUp &&
                   this._hasManufacturerBeenConfirmed &&
                   this._hasModelBeenConfirmed;
        }

        /// <summary>
        /// Sends the disconnection request to the current MWS.
        /// </summary>
        private partial void SendDisconnectionRequest();

        /// <inheritdoc/>
        public void Disconnect()
        {
            this.SendDisconnectionRequest();
        }

        /// <summary>
        /// Disconnects from the MWS assuming the device is invalid.
        /// </summary>
        private void DisconnectForInvalidDevice() 
        {
            this.SendDisconnectionRequest();
            this._services.EventService.InvokeSystemNotificationDispatchedEvent(this,
                new Interfaces.SystemNotificationEventArgs("Invalid Device",
                                                           "Connected device not an MWS.",
                                                           category: Interfaces.NotificationCategory.Error,
                                                           isUrgent: true));
        }

        /// <summary>
        /// Performs required cleanup on disconnect.
        /// </summary>
        public void OnDisconnect() 
        {
            App.Current.Dispatcher.Dispatch(() =>
            {
                this._services.ScanningService.Reset();
                this.CurrentConnection.IsConnected = false;
                this.CurrentConnection = null;
                this._isConnectionSetUp = false;
                this._hasManufacturerBeenConfirmed = false;
                this._hasModelBeenConfirmed = false;
            });
        }

        /// <summary>
        /// Reads the value from the given service and characteristic from the MWS.
        /// </summary>
        /// <param name="serviceUuid">
        /// The UUID for the service.
        /// </param>
        /// <param name="characteristicUuid">
        /// The UUID for the characteristic.
        /// </param>
        private partial void ReadValue(string serviceUuid, string characteristicUuid);

        /// <summary>
        /// Runs the callback on reading a value from the MWS.
        /// </summary>
        /// <param name="uuid">
        /// The UUID of the value.
        /// </param>
        /// <param name="value">
        /// The bytes that make up the value.
        /// </param>
        private void ReadValueCallback(string uuid, byte[] value) 
        {
            // Check which service is read.
            if(uuid.ToUpper() == MwsUuidStrings.DeviceManufacturerUuid)
            {
                // Handle the read of the manufacturer.
                char[] manufacturerString = Encoding.ASCII.GetChars(value);
                string manufacturer = new string(manufacturerString);
                if(manufacturer != "Senior Design Fall 2023")
                {
                    // The device does not fit the interface of an MWS.
                    this.DisconnectForInvalidDevice();
                }
                else
                {
                    this._hasManufacturerBeenConfirmed = true;

                    // Read the model next.
                    this.ReadValue(MwsUuidStrings.DeviceServiceUuid, MwsUuidStrings.DeviceModelUuid);
                }
            }
            else if(uuid.ToUpper() == MwsUuidStrings.DeviceModelUuid)
            {
                // Handle the read of the model.
                char[] modelString = Encoding.ASCII.GetChars(value);
                string model = new string(modelString);
                if(model != "MWS")
                {
                    // The device does not fit the interface of an MWS.
                    this.DisconnectForInvalidDevice();
                }
                else
                {
                    this._hasModelBeenConfirmed = true;
                }
            }
            else if(uuid.ToUpper() == MwsUuidStrings.CalibrationOneUuid) 
            {
                // Save the value.
                string appData = FileSystem.Current.AppDataDirectory;
                string filePath = Path.Combine(appData, this.CurrentConnection.Name);
                if(!File.Exists(filePath))
                {
                    using(StreamWriter file = new StreamWriter(File.Create(filePath)))
                    {
                        file.Write(value[0] + "\n\n\n\n");
                    }
                    
                }
                else
                {
                    string fileContents = File.ReadAllText(filePath);
                    string[] lines = fileContents.Split('\n');
                    lines[0] = value[0].ToString();
                    File.WriteAllText(filePath, lines.Take(4).Aggregate("", (c, n) => c + n + "\n"));
                }

                // Get the next UUID.
                this.ReadValue(MwsUuidStrings.CalibrationServiceUuid, MwsUuidStrings.CalibrationTwoUuid);
            }
            else if(uuid.ToUpper() == MwsUuidStrings.CalibrationTwoUuid)
            {
                // Save the value.
                string appData = FileSystem.Current.AppDataDirectory;
                string filePath = Path.Combine(appData, this.CurrentConnection.Name);
                string fileContents = File.ReadAllText(filePath);
                string[] lines = fileContents.Split('\n');
                lines[1] = value[0].ToString();
                File.WriteAllText(filePath, lines.Take(4).Aggregate("", (c, n) => c + n + "\n"));

                // Get the next UUID.
                this.ReadValue(MwsUuidStrings.CalibrationServiceUuid, MwsUuidStrings.CalibrationThreeUuid);
            }
            else if(uuid.ToUpper() == MwsUuidStrings.CalibrationThreeUuid)
            {
                // Save the value.
                string appData = FileSystem.Current.AppDataDirectory;
                string filePath = Path.Combine(appData, this.CurrentConnection.Name);
                string fileContents = File.ReadAllText(filePath);
                string[] lines = fileContents.Split('\n');
                lines[2] = value[0].ToString();
                File.WriteAllText(filePath, lines.Take(4).Aggregate("", (c, n) => c + n + "\n"));

                // Get the next UUID.
                this.ReadValue(MwsUuidStrings.CalibrationServiceUuid, MwsUuidStrings.CalibrationFourUuid);
            }
            else if(uuid.ToUpper() == MwsUuidStrings.CalibrationFourUuid)
            {
                // Save the value.
                string appData = FileSystem.Current.AppDataDirectory;
                string filePath = Path.Combine(appData, this.CurrentConnection.Name);
                string fileContents = File.ReadAllText(filePath);
                string[] lines = fileContents.Split('\n');
                lines[3] = value[0].ToString();
                File.WriteAllText(filePath, lines.Take(4).Aggregate("", (c, n) => c + n + "\n"));
            }
        }

        /// <summary>
        /// Writes the value to the specified service characteristic and waits for it to be done.
        /// </summary>
        /// <param name="serviceUuid">
        /// The UUID of the service.
        /// </param>
        /// <param name="characteristicUuid">
        /// The UUID of the characteristic.
        /// </param>
        /// <param name="value">
        /// The value to be written.
        /// </param>
        /// <returns>
        /// A task representing the asynchronous state of the operation.
        /// </returns>
        private partial Task WriteValue(string serviceUuid, string characteristicUuid, byte[] value);

        /// <summary>
        /// Processes a change notification on the given UUID.
        /// </summary>
        /// <param name="uuid">
        /// The UUID of the characteristic that was changed.
        /// </param>
        /// <param name="newValue">
        /// The new value of the characteristic.
        /// </param>
        private void ProcessNotification(string uuid, byte[] newValue)
        {
            if(uuid.ToUpper() == MwsUuidStrings.MessengerMessageUuid)
            {
                // Notify the user of the message.
                this._services.EventService.InvokeSystemNotificationDispatchedEvent(this,
                                                                                    new SystemNotificationEventArgs("MWS Message",
                                                                                                                    Encoding.ASCII.GetString(newValue),
                                                                                                                    category: NotificationCategory.Status,
                                                                                                                    isUrgent: true));
            }
            else if(uuid.ToUpper() == MwsUuidStrings.FollowerChangeAdvertisementUuid)
            {
                // Change the advertisement.
                this.StopAdvertisingLocation();
                this.GenerateSignature();
                this.StartAdvertisingLocation();

                // Send an acknowledgement.
                this.WriteValue(MwsUuidStrings.FollowerServiceUuid, MwsUuidStrings.FollowerAcknowledgeUuid, new byte[] { 0x01 });
            }
            else if(uuid.ToUpper() == MwsUuidStrings.CalibrationIsCalibratingUuid) 
            {
                // Read the new calibration values for later. Start with one and the rest are chained.
                App.Current.Dispatcher.Dispatch(() => 
                {
                    this.CurrentConnection.IsCalibrating = false;
                    this.ReadValue(MwsUuidStrings.CalibrationServiceUuid, MwsUuidStrings.CalibrationOneUuid);
                });
            }
        }

        /// <inheritdoc/>
        public async Task Calibrate(byte target) 
        {
            await this.WriteValue(MwsUuidStrings.CalibrationServiceUuid, MwsUuidStrings.CalibrationTargetUuid, new byte[] { target });
            await this.WriteValue(MwsUuidStrings.CalibrationServiceUuid, MwsUuidStrings.CalibrationIsCalibratingUuid, new byte[] { 0x01 });
        }

        /// <summary>
        /// Sends a request to rename the MWS.
        /// </summary>
        /// <param name="name">
        /// The name to give the MWS.
        /// </param>
        /// <returns>
        /// Whether th eoperation was successful.
        /// </returns>
        private partial bool SendRenameRequest(string name);

        /// <inheritdoc/>
        public bool Rename(string name)
        {
            if(this.SendRenameRequest(name))
            {
                this.CurrentConnection.Name = name;
                return true;
            }
            else 
            {
                return false;
            }
        }

        /// <summary>
        /// Generates a signature.
        /// </summary>
        /// <returns>
        /// The bytes representing the signature.
        /// </returns>
        private partial byte[] GenerateSignature();

        /// <summary>
        /// Runs the actions required to place the MWS in the state where it will look for location and begins sharing location.
        /// </summary>
        /// <returns>
        /// Whether the location sharing successfully started.
        /// </returns>
        private partial bool StartAdvertisingLocation();

        /// <inheritdoc/>
        public async Task EnterFollowerMode()
        {
            // Write to the signature characteristic.
            if(!(this.GenerateSignature() is byte[] value))
            {
                this._services.EventService.InvokeUIMessageDispatchedEvent(this, new UIMessageEventArgs("Error placing " + this.CurrentConnection.Name + " into follower mode"));
            }
            else
            {
                await this.WriteValue(MwsUuidStrings.FollowerServiceUuid, MwsUuidStrings.FollowerSignatureUuid, value);

                // Turn follower mode on. Write little-endian 1 for true.
                await this.WriteValue(MwsUuidStrings.FollowerServiceUuid, MwsUuidStrings.FollowerIsActivatedUuid, new byte[] { 0x01, 0x00, 0x00, 0x00 });

                // Start the advertisement.
                if(this.StartAdvertisingLocation())
                {
                    // Update the display.
                    this.CurrentConnection.IsFollowing = true;
                }
                else
                {
                    // Notify about the error.
                    this._services.EventService.InvokeUIMessageDispatchedEvent(this, new UIMessageEventArgs("Error placing " + this.CurrentConnection.Name + " into follower mode"));
                }
            }
        }

        /// <summary>
        /// Stops the process of sharing location.
        /// </summary>
        private partial void StopAdvertisingLocation();

        /// <inheritdoc/>
        public async Task ExitFollowerMode()
        {
            // Turn follower mode off. Write little-endian 0 for false.
            await this.WriteValue(MwsUuidStrings.FollowerServiceUuid, MwsUuidStrings.FollowerIsActivatedUuid, new byte[] { 0x00, 0x00, 0x00, 0x00 });

            // Stop advertising.
            this.StopAdvertisingLocation();

            // Update the UI.
            this.CurrentConnection.IsFollowing = false;
        }

        /// <summary>
        /// Raises the property changed event in a streamlined manner.
        /// </summary>
        /// <param name="propertyName">
        /// The name of the property that has changed. This defaults to the clling member's name.
        /// </param>
        protected void RaisePropertyChanged([CallerMemberName] string propertyName = null)
        {
            App.Current.Dispatcher.Dispatch(() => this.PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName)));
        }

        /// <summary>
        /// Clears the current connection after a disconnect.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BluetoothChangedHandler(object sender, bool e) 
        {
            // Clear the current connection on disconnect.
            if(this.CurrentConnection != null)
            {
                this.CurrentConnection.IsConnected = false;
                this.CurrentConnection = null;
            }
        }
    }

    /// <summary>
    /// Contains the strings of the UUIDs of the services and characteristics.
    /// </summary>
    public static class MwsUuidStrings
    {
        /// <summary>
        /// The UUID advertised by the MWS.
        /// </summary>
        public const string AdvertiseUuid = "CCA85698-A7BE-4E5A-8506-9125CE3D98E8";

        /// <summary>
        /// The UUID of the device service.
        /// </summary>
        public const string DeviceServiceUuid = "C76C0000-4F43-40E7-A27E-ADFBD971EC35";

        /// <summary>
        /// The UUID of the mfgr_name characteristic of the device service.
        /// </summary>
        public const string DeviceManufacturerUuid = "C76C2A29-4F43-40E7-A27E-ADFBD971EC35";

        /// <summary>
        /// The UUID of the model_num characteristic of the device service.
        /// </summary>
        public const string DeviceModelUuid = "C76C2A24-4F43-40E7-A27E-ADFBD971EC35";

        /// <summary>
        /// The UUID of the battery service.
        /// </summary>
        public const string BatteryServiceUuid = "0000180F-0000-1000-8000-00805F9B34FB";

        /// <summary>
        /// The UUID of the level characteristic of the battery service.
        /// </summary>
        public const string BatteryLevelUuid = "00002A19-0000-1000-8000-00805F9B34FB";

        /// <summary>
        /// The UUID of the follower service.
        /// </summary>
        public const string FollowerServiceUuid = "CCA80000-A7BE-4E5A-8506-9125CE3D98E8";

        /// <summary>
        /// The UUID of the is_activated characteristic of the follower service.
        /// </summary>
        public const string FollowerIsActivatedUuid = "CCA80001-A7BE-4E5A-8506-9125CE3D98E8";

        /// <summary>
        /// The UUID of the signature characteristic of the follower service.
        /// </summary>
        public const string FollowerSignatureUuid = "CCA80002-A7BE-4E5A-8506-9125CE3D98E8";

        /// <summary>
        /// The UUID of the change_advertisement characteristic of the follower service.
        /// </summary>
        public const string FollowerChangeAdvertisementUuid = "CCA80003-A7BE-4E5A-8506-9125CE3D98E8";

        /// <summary>
        /// The UUID of the range characteristic of the follower service.
        /// </summary>
        public const string FollowerRangeUuid = "CCA80004-A7BE-4E5A-8506-9125CE3D98E8";

        /// <summary>
        /// The UUID of the acknowledge characteristic of the follower service.
        /// </summary>
        public const string FollowerAcknowledgeUuid = "CCA80005-A7BE-4E5A-8506-9125CE3D98E8";

        /// <summary>
        /// The UUID of the messenger service.
        /// </summary>
        public const string MessengerServiceUuid = "A58F0000-6AD9-4E68-8333-BFEF89ADCF9B";

        /// <summary>
        /// The UUID of the message characteristic of the messenger service.
        /// </summary>
        public const string MessengerMessageUuid = "A58F0001-6AD9-4E68-8333-BFEF89ADCF9B";

        /// <summary>
        /// The UUID to put into the client advertisement.
        /// </summary>
        public const string ClientAdvertisementUuid = "560D5EE7-2C11-478F-AB58-07211516C261";

        /// <summary>
        /// The UUID of the calibration service.
        /// </summary>
        public const string CalibrationServiceUuid = "3C020000-295C-4EF0-8621-2E2AEA667070";

        /// <summary>
        /// The UUID of the value for number One.
        /// </summary>
        public const string CalibrationOneUuid = "3C020001-295C-4EF0-8621-2E2AEA667070";

        /// <summary>
        /// The UUID of the value for number Two.
        /// </summary>
        public const string CalibrationTwoUuid = "3C020002-295C-4EF0-8621-2E2AEA667070";

        /// <summary>
        /// The UUID of the value for number Three.
        /// </summary>
        public const string CalibrationThreeUuid = "3C020003-295C-4EF0-8621-2E2AEA667070";

        /// <summary>
        /// The UUID of the value for number Four.
        /// </summary>
        public const string CalibrationFourUuid = "3C020004-295C-4EF0-8621-2E2AEA667070";

        /// <summary>
        /// The UUID for whether calibration is occurring.
        /// </summary>
        public const string CalibrationIsCalibratingUuid = "3C020005-295C-4EF0-8621-2E2AEA667070";

        /// <summary>
        /// The UUID for the target to calibrate.
        /// </summary>
        public const string CalibrationTargetUuid = "3C020006-295C-4EF0-8621-2E2AEA667070";
    }
}
