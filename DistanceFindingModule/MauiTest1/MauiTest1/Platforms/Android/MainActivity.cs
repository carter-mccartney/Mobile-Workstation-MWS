using Android.App;
using Android.Content.PM;
using Android.OS;
using AndroidX.Activity.Result;
using MauiTest1.Services;
using AndroidX.Activity.Result.Contract;
using Java.Interop;
using Android.Content;
using System.Runtime.CompilerServices;
using Microsoft.Maui.Controls.PlatformConfiguration;
using Android.Bluetooth;

namespace MauiTest1;

[Activity(Theme = "@style/Maui.SplashTheme", MainLauncher = true, ConfigurationChanges = ConfigChanges.ScreenSize | ConfigChanges.Orientation | ConfigChanges.UiMode | ConfigChanges.ScreenLayout | ConfigChanges.SmallestScreenSize | ConfigChanges.Density)]
public class MainActivity : MauiAppCompatActivity
{
    public static ActivityResultLauncher Launcher;
    public static MainActivity Instance { get; private set; }
    private static bool _isBluetoothOn;
    public static bool IsBluetoothOn 
    { 
        get => MainActivity._isBluetoothOn;
        private set 
        { 
            MainActivity._isBluetoothOn = value;
            MainActivity.IsBluetoothOnChangedEvent?.Invoke(MainActivity.Instance, new EventArgs());
        }
    }
    public static event EventHandler IsBluetoothOnChangedEvent;
    public MainActivity() 
    { 
        MainActivity.Instance = this;
        MainActivity.Launcher = this.RegisterForActivityResult(new ActivityResultContracts.RequestMultiplePermissions(), ActivityResultCallbackSubclass.Instance);
    }

    public void Initialize() 
    {
        this.RegisterReceiver(new BluetoothDisableReceiver(), new IntentFilter(BluetoothAdapter.ActionStateChanged));
        BluetoothManager bleManager = (BluetoothManager)MainActivity.Instance.GetSystemService(MainActivity.BluetoothService);
        MainActivity.IsBluetoothOn = bleManager.Adapter != null;
    }

    protected override void OnActivityResult(int requestCode, Result resultCode, Intent data)
    {
        base.OnActivityResult(requestCode, resultCode, data);

        if(requestCode == 1) 
        {
            MainActivity.IsBluetoothOn = resultCode == Result.Ok;
        }
    }

    private class ActivityResultCallbackSubclass : Java.Lang.Object, IActivityResultCallback
    {
        public static ActivityResultCallbackSubclass Instance = new ActivityResultCallbackSubclass();

        public void OnActivityResult(Java.Lang.Object p0)
        {
        }
    }

    private class BluetoothDisableReceiver : BroadcastReceiver
    {
        public override void OnReceive(Context context, Intent intent)
        {
            if(intent.Action == Android.Bluetooth.BluetoothAdapter.ActionStateChanged)
            {
                Android.Bluetooth.State state = (Android.Bluetooth.State)intent.GetIntExtra(Android.Bluetooth.BluetoothAdapter.ExtraState, Android.Bluetooth.BluetoothAdapter.Error);
                MainActivity.IsBluetoothOn = state == Android.Bluetooth.State.On;
            }
        }
    }
}