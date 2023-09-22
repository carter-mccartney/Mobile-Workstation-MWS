using MwsCompanionApp.Interfaces;
using MwsCompanionApp.Services;
using MwsCompanionApp.Views;

namespace MwsCompanionApp;

/// <summary>
/// Represents the entire app.
/// </summary>
public partial class App : Application
{
    /// <summary>
    /// The services for the application
    /// </summary>
    private readonly ServicesContainer _services;

    /// <summary>
    /// Creates an instance of the app.
    /// </summary>
    public App(ServicesContainer services)
    {
        this._services = services;

        this.InitializeComponent();
		this.MainPage = new AppShell(services);
    }

    /// <summary>
    /// Resumes the app.
    /// </summary>
    protected override void OnResume()
    {
        base.OnResume();

        // Update theme on resume.
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

        // Notify listeners of a resume.
        this._services.EventService.InvokeResumedEvent(this, new EventArgs());
    }

    /// <summary>
    /// Puts the app to sleep.
    /// </summary>
    protected override void OnSleep()
    {
        base.OnSleep();
        this._services.EventService.InvokeSleepStarted(this, new EventArgs());
    }

    /// <summary>
    /// Cleans up the application state on close.
    /// </summary>
    protected override void CleanUp()
    {
        base.CleanUp();
        this._services.EventService.InvokeAppClosedEvent(this, new EventArgs());
    }
}
