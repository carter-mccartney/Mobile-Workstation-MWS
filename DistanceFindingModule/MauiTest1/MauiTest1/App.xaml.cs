using MauiTest1.Views;
using Microsoft.Maui.Controls;

namespace MauiTest1;

public partial class App : Application
{

    public static event EventHandler Resumed;

	public App()
	{
		this.InitializeComponent();

		this.MainPage = new AppShell();
	}

    protected override void OnResume()
    {
        base.OnResume();
        string currentTheme = Preferences.Default.Get("Theme", "Unspecified");
        if(currentTheme == "Unspecified")
        {
            Application.Current.UserAppTheme = Application.Current.PlatformAppTheme;
        }
        else if(currentTheme == "Light")
        {
            Application.Current.UserAppTheme = AppTheme.Light;
        }
        else
        {
            Application.Current.UserAppTheme = AppTheme.Dark;
        }
        App.Resumed?.Invoke(this, new EventArgs());
    }
}
