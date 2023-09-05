using Android;
using Android.App;
using Android.Bluetooth;
using Android.Bluetooth.LE;
using Android.Content;
using Android.OS;
using Android.Runtime;
using AndroidX.Activity.Result.Contract;
using AndroidX.Annotations;
using AndroidX.Core.Content;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MauiTest1.Services
{
    [Service]
    public partial class BackgroundService : Service
    {

        public partial void Initialize()
        {
            MainActivity.IsBluetoothOnChangedEvent += this.BluetoothChangedHandler;

            // Check for permissions.
            if(ContextCompat.CheckSelfPermission(MainActivity.Instance, Manifest.Permission.BluetoothAdvertise) != Android.Content.PM.Permission.Granted ||
               ContextCompat.CheckSelfPermission(MainActivity.Instance, Manifest.Permission.BluetoothConnect) != Android.Content.PM.Permission.Granted ||
               ContextCompat.CheckSelfPermission(MainActivity.Instance, Manifest.Permission.Bluetooth) != Android.Content.PM.Permission.Granted)
            {
                MainActivity.Launcher.Launch(new string[] { Manifest.Permission.Bluetooth, Manifest.Permission.BluetoothConnect, Manifest.Permission.BluetoothAdvertise });
                if(ContextCompat.CheckSelfPermission(MainActivity.Instance, Manifest.Permission.BluetoothAdvertise) == Android.Content.PM.Permission.Granted &&
                   ContextCompat.CheckSelfPermission(MainActivity.Instance, Manifest.Permission.BluetoothConnect) == Android.Content.PM.Permission.Granted &&
                   ContextCompat.CheckSelfPermission(MainActivity.Instance, Manifest.Permission.Bluetooth) == Android.Content.PM.Permission.Granted)
                {
                    // Check if bluetooth is on.
                    BluetoothManager bleManager = (BluetoothManager)MainActivity.Instance.GetSystemService(MainActivity.BluetoothService);
                    if(!bleManager.Adapter.IsEnabled)
                    {
                        Intent enableBTIntent = new Intent(BluetoothAdapter.ActionRequestEnable);
                        MainActivity.Instance.StartActivityForResult(enableBTIntent, 1);
                    }
                    MainActivity.Instance.Initialize();
                }
            }
            else
            {
                // Check if bluetooth is on.
                BluetoothManager bleManager = (BluetoothManager)MainActivity.Instance.GetSystemService(MainActivity.BluetoothService);
                if(!bleManager.Adapter.IsEnabled)
                {
                    Intent enableBTIntent = new Intent(BluetoothAdapter.ActionRequestEnable);
                    MainActivity.Instance.StartActivityForResult(enableBTIntent, 1);
                }
                MainActivity.Instance.Initialize();
            }
            this.IsBluetoothOn = MainActivity.IsBluetoothOn;
        }

        public override IBinder OnBind(Intent intent)
        {
            throw new NotImplementedException();
        }

        [return: GeneratedEnum]
        public override StartCommandResult OnStartCommand(Intent intent, [GeneratedEnum] StartCommandFlags flags, int startId)
        {
            if(intent.Action == "START_SERVICE")
            {
                BluetoothManager bleManager = (BluetoothManager)this.GetSystemService(MainActivity.BluetoothService);
                BluetoothLeAdvertiser advertiser = bleManager.Adapter.BluetoothLeAdvertiser;
                AdvertisingSetParameters advertisingParameters = (new AdvertisingSetParameters.Builder())
                                                                    .SetLegacyMode(true)
                                                                    .SetScannable(true)
                                                                    .SetConnectable(false)
                                                                    .SetInterval(AdvertisingSetParameters.IntervalLow)
                                                                    .SetTxPowerLevel(AdvertiseTxPower.High)
                                                                    .Build();
                AdvertiseData advertiseData = (new AdvertiseData.Builder())
                                                .SetIncludeDeviceName(true)
                                                .SetIncludeTxPowerLevel(true)
                                                .Build();
                advertiser.StartAdvertisingSet(advertisingParameters, advertiseData, null, null, null, AdvertisingSetCallbackSubclass.Instance);

            }
            else if(intent.Action == "STOP_SERVICE")
            {
                BluetoothManager bleManager = (BluetoothManager)this.GetSystemService(MainActivity.BluetoothService);
                BluetoothLeAdvertiser advertiser = bleManager.Adapter.BluetoothLeAdvertiser;
                if(advertiser != null)
                {
                    advertiser.StopAdvertisingSet(AdvertisingSetCallbackSubclass.Instance);
                }
                if(!OperatingSystem.IsAndroidVersionAtLeast(33))
                {
                    this.StopForeground(true);
                }
                this.StopSelfResult(startId);
            }

            return StartCommandResult.NotSticky;
        }


        public partial void Start()
        {
            Intent startService = new Intent(MainActivity.Instance, typeof(BackgroundService));
            startService.SetAction("START_SERVICE");
            MainActivity.Instance.StartService(startService);
        }

        public partial void Stop()
        {
            Intent stopService = new Intent(MainActivity.Instance, typeof(BackgroundService));
            stopService.SetAction("STOP_SERVICE");
            MainActivity.Instance.StartService(stopService);
        }

        public void BluetoothChangedHandler(object sender, EventArgs e) 
        {
            this.IsBluetoothOn = MainActivity.IsBluetoothOn;
        }
    }

    internal class AdvertisingSetCallbackSubclass : AdvertisingSetCallback
    {
        public static AdvertisingSetCallbackSubclass Instance = new AdvertisingSetCallbackSubclass();

        public override void OnAdvertisingSetStarted(AdvertisingSet advertisingSet, int txPower, [GeneratedEnum] AdvertiseResult status)
        {
            
        }

        public override void OnAdvertisingDataSet(AdvertisingSet advertisingSet, [GeneratedEnum] AdvertiseResult status)
        {

        }

        public override void OnScanResponseDataSet(AdvertisingSet advertisingSet, [GeneratedEnum] AdvertiseResult status)
        {

        }

        public override void OnAdvertisingSetStopped(AdvertisingSet advertisingSet)
        {

        }
    }
}
