using CommunityToolkit.Maui.Alerts;
using MwsCompanionApp.Interfaces;
using MwsCompanionApp.Services;
using Plugin.LocalNotification;

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
        services.EventService.UIMessageDispatched += this.ShowMessage;
        services.EventService.SystemNotificationDispatched += this.ShowNotification;
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
    private void ShowMessage(object sender, UIMessageEventArgs e) 
    {
        Toast.Make(e.Message, e.IsLong ? CommunityToolkit.Maui.Core.ToastDuration.Long : CommunityToolkit.Maui.Core.ToastDuration.Short).Show();
    }

    /// <summary>
    /// Shows a system notification.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event.
    /// </param>
    /// <param name="e">
    /// Context for the event, describing the notification.
    /// </param>
    private async void ShowNotification(object sender, SystemNotificationEventArgs e) 
    { 
        // Build a corresponding notification.
        NotificationRequest request = new NotificationRequest() 
        { 
            CategoryType = e.Category == Interfaces.NotificationCategory.Unspecified ? 
                               NotificationCategoryType.None : 
                               e.Category == Interfaces.NotificationCategory.Error ? 
                                   NotificationCategoryType.Error : 
                                   e.Category == Interfaces.NotificationCategory.Progress ? 
                                       NotificationCategoryType.Progress : 
                                       e.Category == Interfaces.NotificationCategory.Service ? 
                                           NotificationCategoryType.Service : 
                                           e.Category == Interfaces.NotificationCategory.Status ? 
                                               NotificationCategoryType.Status : 
                                               throw new ArgumentOutOfRangeException("NotificationCategory " + e.Category + " is not specified in handler."),
            Description = e.Description,
            Image = new NotificationImage() { FilePath = e.IconPath },
            Group = e.Channel,
            Silent = !e.IsUrgent, // Urgent notifications should make a sound.
            Subtitle = e.Subtitle,
            Title = e.Title
        };

        // Display the notification.
        await request.Show();
    }
}
