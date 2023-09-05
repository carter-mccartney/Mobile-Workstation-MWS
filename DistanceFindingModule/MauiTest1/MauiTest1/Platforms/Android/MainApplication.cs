using Android.App;
using Android.Hardware.Camera2;
using Android.Runtime;

namespace MauiTest1;

[Application]
public class MainApplication : MauiApplication
{

    public MainApplication(IntPtr handle, JniHandleOwnership ownership)
		: base(handle, ownership)
    {
    }

	protected override MauiApp CreateMauiApp() => MauiProgram.CreateMauiApp();
}
