namespace MauiTest1.Views;

public partial class AppShell : Shell
{
	public AppShell()
	{
		this.InitializeComponent();
    }

    private void Shell_Loaded(object sender, EventArgs e)
    {
        string theme = Preferences.Default.Get("Theme", "Unspecified");
        if(theme == "Unspecified")
        {
            Application.Current.UserAppTheme = Application.Current.PlatformAppTheme;
        }
        else if(theme == "Light")
        {
            Application.Current.UserAppTheme = AppTheme.Light;
        }
        else
        {
            Application.Current.UserAppTheme = AppTheme.Dark;
        }
    }
}
