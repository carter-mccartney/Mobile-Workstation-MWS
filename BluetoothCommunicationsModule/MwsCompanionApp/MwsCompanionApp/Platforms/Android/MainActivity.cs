using Android.App;
using Android.Bluetooth;
using Android.Content;
using Android.Content.PM;
using Android.OS;
using AndroidX.Activity.Result;
using AndroidX.Activity.Result.Contract;

namespace MwsCompanionApp;

[Activity(Theme = "@style/Maui.SplashTheme", MainLauncher = true, ConfigurationChanges = ConfigChanges.ScreenSize | ConfigChanges.UiMode | ConfigChanges.ScreenLayout | ConfigChanges.SmallestScreenSize | ConfigChanges.Density)]
public class MainActivity : MauiAppCompatActivity
{
    /// <summary>
    /// Launches permission requests.
    /// </summary>
    private ActivityResultLauncher PermissionsRequester;

    /// <summary>
    /// The instance of the activity for the application.
    /// </summary>
    public static MainActivity Instance { get; private set; }

    private static bool _isBluetoothOn;
    /// <summary>
    /// Whether bluetooth is enabled.
    /// </summary>
    public static bool IsBluetoothOn
    {
        get => MainActivity._isBluetoothOn;
        private set
        {
            MainActivity._isBluetoothOn = value;
            MainActivity.IsBluetoothOnChangedEvent?.Invoke(MainActivity.Instance, new EventArgs());
        }
    }

    /// <summary>
    /// Whether the bluetooth is accessible.
    /// </summary>
    public static event EventHandler IsBluetoothOnChangedEvent;

    /// <summary>
    /// Creates the main activity and registers the static instances.
    /// </summary>
    public MainActivity()
    {
        MainActivity.Instance = this;
        this.PermissionsRequester = this.RegisterForActivityResult(new ActivityResultContracts.RequestMultiplePermissions(), PermissionsRequestCallback.Instance);
    }

    /// <summary>
    /// Initializes the bluetooth state.
    /// </summary>
    public void Initialize()
    {
        this.RegisterReceiver(new BluetoothStateReceiver(), new IntentFilter(BluetoothAdapter.ActionStateChanged));
        BluetoothManager bleManager = (BluetoothManager)MainActivity.Instance.GetSystemService(MainActivity.BluetoothService);
        MainActivity.IsBluetoothOn = bleManager.Adapter != null;
    }

    /// <summary>
    /// Requests the permissions in the string.
    /// </summary>
    /// <param name="permissions">
    /// The permissions to request.
    /// </param>
    /// <param name="callback">
    /// The action to follow the completion of the request.
    /// </param>
    public void LaunchPermissionRequest(string[] permissions, Action callback) 
    {
        PermissionsRequestCallback.Instance.Callback = callback;
        this.PermissionsRequester.Launch(permissions);
    }

    /// <summary>
    /// Handler for any user interaction from dialogs.
    /// </summary>
    /// <param name="requestCode">
    /// The type of action.
    /// </param>
    /// <param name="resultCode">
    /// The result of the action.
    /// </param>
    /// <param name="data">
    /// Extra data about the action.
    /// </param>
    protected override void OnActivityResult(int requestCode, Result resultCode, Intent data)
    {
        base.OnActivityResult(requestCode, resultCode, data);

        if(requestCode == 1)
        {
            MainActivity.IsBluetoothOn = resultCode == Result.Ok;
        }
    }

    /// <summary>
    /// Internal class for handling the permission request.
    /// </summary>
    private class PermissionsRequestCallback : Java.Lang.Object, IActivityResultCallback
    {
        /// <summary>
        /// The callback to run after requesting permissions.
        /// </summary>
        public Action Callback;

        /// <summary>
        /// The singular instance of this callback.
        /// </summary>
        public static PermissionsRequestCallback Instance = new PermissionsRequestCallback();

        /// <summary>
        /// The action to perform on the result of the activity.
        /// </summary>
        /// <param name="p0">
        /// The result? of the activity.
        /// </param>
        public void OnActivityResult(Java.Lang.Object p0)
        {
            // Run the callback.
            this.Callback?.Invoke();
        }
    }

    /// <summary>
    /// Monitors the state of bluetooth in the OS.
    /// </summary>
    private class BluetoothStateReceiver : BroadcastReceiver
    {
        /// <summary>
        /// Handles a change in the bluetooth state.
        /// </summary>
        /// <param name="context">
        /// The context in which the state change occurred.
        /// </param>
        /// <param name="intent">
        /// Contains information about the state.
        /// </param>
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
