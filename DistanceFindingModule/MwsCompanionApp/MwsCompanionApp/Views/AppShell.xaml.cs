using CommunityToolkit.Maui.Alerts;
using MwsCompanionApp.Interfaces;
using MwsCompanionApp.Services;

namespace MwsCompanionApp.Views;

/// <summary>
/// Represents the shell of the application.
/// </summary>
public partial class AppShell : Shell
{
    /// <summary>
    /// The services for the application.
    /// </summary>
    private ServicesContainer _services;

    /// <summary>
    /// Creates an app shell.
    /// </summary>
    public AppShell(ServicesContainer services)
    {
        this.InitializeComponent();
        this._services = services;
        services.EventService.UIMessageDispatched += this.ShowNotification;
    }

    /// <summary>
    /// Sets the theme on initial load.
    /// </summary>
    /// <param name="sender">
    /// the object invoking the event. This should be the app shell.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored in this handler.
    /// </param>
    private void Shell_Loaded(object sender, EventArgs e)
    {
        // Initialize services.
        _ = this._services.InitializeServices();

        // Read the theme and set it.
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

    /// <summary>
    /// Shows a toast notification.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event.
    /// </param>
    /// <param name="e">
    /// Context for the event, including the message.
    /// </param>
    private void ShowNotification(object sender, UIMessageEventArgs e) 
    {
        Toast.Make(e.Message, e.IsLong ? CommunityToolkit.Maui.Core.ToastDuration.Long : CommunityToolkit.Maui.Core.ToastDuration.Short).Show();
    }
}
