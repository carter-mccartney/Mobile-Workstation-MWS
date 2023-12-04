using MwsCompanionApp.Interfaces;

namespace MwsCompanionApp.Views;

/// <summary>
/// Represents a page for adjusting settings.
/// </summary>
public partial class SettingsPage : ContentPage
{
    /// <summary>
    /// A reference to the application-wide event service.
    /// </summary>
    private IEventService _eventService;

    #region Page Lifecycle

    /// <summary>
    /// Creates a page for adjusting settings.
    /// </summary>
    /// <param name="eventService">
    /// The event service instance to inject.
    /// </param>
    public SettingsPage(IEventService eventService)
    {
        this.InitializeComponent();
        this._eventService = eventService;
    }

    /// <summary>
    /// Registers the resumed event and opens the page properly.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This should be the page.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored in this handler.
    /// </param>
    private void ContentPage_Loaded(object sender, EventArgs e)
    {
        this._eventService.Resumed += this.OpenPage;
        this.OpenPage(sender, e);
    }

    /// <summary>
    /// Unloads the page and deregisters the resumed event.
    /// </summary>
    /// <param name="sender">
    /// THe object invoking the event. This should be the page.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored by this handler.
    /// </param>
    private void ContentPage_Unloaded(object sender, EventArgs e)
    {
        this._eventService.Resumed -= this.OpenPage;
    }

    #endregion

    #region Event Handlers

    /// <summary>
    /// Places accurate settings in the display.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This should be the page.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored in this handler.
    /// </param>
    private void OpenPage(object sender, EventArgs e)
    {
        // Check what settings are saved.
        string currentTheme = Preferences.Default.Get("Theme", "Unspecified");
        if(currentTheme == "Unspecified")
        {
            // If unspecified, go with system theme.
            this.SystemThemeCheckBox.CheckedChanged -= this.CheckBox_CheckedChanged;
            this.SystemThemeCheckBox.IsChecked = true;
            this.SystemThemeCheckBox.CheckedChanged += this.CheckBox_CheckedChanged;
            this.ThemeSwitch.IsEnabled = false;
            this.ThemeSwitch.Toggled -= this.Switch_Toggled;
            this.ThemeSwitch.IsToggled = Application.Current.PlatformAppTheme == AppTheme.Light;
            this.ThemeSwitch.Toggled += this.Switch_Toggled;
            if(this.ThemeSwitch.IsToggled)
            {
                this.CurrentThemeLabel.Text = "Light";
            }
            else
            {
                this.CurrentThemeLabel.Text = "Dark";
            }
        }
        else if(currentTheme == "Light")
        {
            // Set the light theme.
            this.SystemThemeCheckBox.CheckedChanged -= this.CheckBox_CheckedChanged;
            this.SystemThemeCheckBox.IsChecked = false;
            this.SystemThemeCheckBox.CheckedChanged += this.CheckBox_CheckedChanged;
            this.ThemeSwitch.Toggled -= this.Switch_Toggled;
            this.ThemeSwitch.IsToggled = true;
            this.ThemeSwitch.Toggled += this.Switch_Toggled;
            this.CurrentThemeLabel.Text = "Light";
        }
        else
        {
            // Set the dark theme.
            this.SystemThemeCheckBox.CheckedChanged -= this.CheckBox_CheckedChanged;
            this.SystemThemeCheckBox.IsChecked = false;
            this.SystemThemeCheckBox.CheckedChanged += this.CheckBox_CheckedChanged;
            this.ThemeSwitch.Toggled -= this.Switch_Toggled;
            this.ThemeSwitch.IsToggled = false;
            this.ThemeSwitch.Toggled += this.Switch_Toggled;
            this.CurrentThemeLabel.Text = "Dark";
        }
    }

    /// <summary>
    /// Alternates the system theme setting.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This should be the checkbox for the system theme.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored in this handler.
    /// </param>
    private void CheckBox_CheckedChanged(object sender, CheckedChangedEventArgs e)
    {
        // Check current state.
        if(this.SystemThemeCheckBox.IsChecked)
        {
            // If set, change to system theme and adjust view.
            Application.Current.UserAppTheme = Application.Current.PlatformAppTheme;
            Preferences.Default.Set("Theme", "Unspecified");
            this.ThemeSwitch.IsEnabled = false;
            this.ThemeSwitch.Toggled -= this.Switch_Toggled;
            this.ThemeSwitch.IsToggled = Application.Current.PlatformAppTheme == AppTheme.Light;
            this.ThemeSwitch.Toggled += this.Switch_Toggled;
            if(this.ThemeSwitch.IsToggled)
            {
                this.CurrentThemeLabel.Text = "Light";
            }
            else
            {
                this.CurrentThemeLabel.Text = "Dark";
            }
        }
        else
        {
            // If not set, disable the selection of themes and set the theme.
            this.ThemeSwitch.IsEnabled = true;
            this.SetTheme();
        }
    }

    /// <summary>
    /// Alternates the theme between light and dark mode.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This should be the theme switch.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored in this handler.
    /// </param>
    private void Switch_Toggled(object sender, ToggledEventArgs e)
    {
        this.SetTheme();
    }

    #endregion

    #region Helpers

    /// <summary>
    /// Changes the theme based on the state of the theme switch.
    /// </summary>
    private void SetTheme()
    {
        if(this.ThemeSwitch.IsToggled)
        {
            // Use the light theme.
            Application.Current.UserAppTheme = AppTheme.Light;
            Preferences.Default.Set("Theme", "Light");
            this.CurrentThemeLabel.Text = "Light";
        }
        else
        {
            // Use the dark theme.
            Application.Current.UserAppTheme = AppTheme.Dark;
            Preferences.Default.Set("Theme", "Dark");
            this.CurrentThemeLabel.Text = "Dark";
        }
    }

    #endregion
}