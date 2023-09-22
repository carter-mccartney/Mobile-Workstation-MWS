using Android;
using Android.Bluetooth;
using Android.Content;
using AndroidX.Core.Content;
using MwsCompanionApp.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Services
{
    public partial class PermissionsService
    {
        public partial void CheckPermissions()
        {
            Task.Run(() =>
            {
                if(ContextCompat.CheckSelfPermission(MainActivity.Instance, Manifest.Permission.BluetoothAdvertise) != Android.Content.PM.Permission.Granted ||
                   ContextCompat.CheckSelfPermission(MainActivity.Instance, Manifest.Permission.BluetoothConnect) != Android.Content.PM.Permission.Granted ||
                   ContextCompat.CheckSelfPermission(MainActivity.Instance, Manifest.Permission.Bluetooth) != Android.Content.PM.Permission.Granted)
                {
                    MainActivity.Instance.LaunchPermissionRequest(new string[]
                                                                  {
                                                                  Manifest.Permission.Bluetooth,
                                                                  Manifest.Permission.BluetoothConnect,
                                                                  Manifest.Permission.BluetoothAdvertise
                                                                  },
                                                                  () =>
                    {
                        Task.Run(() =>
                        {
                            if(ContextCompat.CheckSelfPermission(MainActivity.Instance, Manifest.Permission.BluetoothAdvertise) != Android.Content.PM.Permission.Granted ||
                               ContextCompat.CheckSelfPermission(MainActivity.Instance, Manifest.Permission.BluetoothConnect) != Android.Content.PM.Permission.Granted ||
                               ContextCompat.CheckSelfPermission(MainActivity.Instance, Manifest.Permission.Bluetooth) != Android.Content.PM.Permission.Granted)
                            {
                                this._services.EventService.InvokeUIMessageDispatchedEvent(this, new UIMessageEventArgs("Bluetooth permissions are required for application", true));
                                this.IsBluetoothPermitted = false;
                            }
                            else
                            {
                                this.IsBluetoothPermitted = true;
                            }
                        });
                    });
                }
                else
                {
                    this.IsBluetoothPermitted = true;
                }
            });
        }
    }
}
