namespace MauiTest1.Views;

public partial class SettingsPage : ContentPage
{
	public SettingsPage()
    {
        this.InitializeComponent();
    }

    private void ContentPage_Loaded(object sender, EventArgs e)
    {
        App.Resumed += this.OpenPage;
        this.OpenPage(sender, e);
    }


    private void OpenPage(object sender, EventArgs e)
    {
        string currentTheme = Preferences.Default.Get("Theme", "Unspecified");
        if(currentTheme == "Unspecified")
        {
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
            this.SystemThemeCheckBox.CheckedChanged -= this.CheckBox_CheckedChanged;
            this.SystemThemeCheckBox.IsChecked = false;
            this.SystemThemeCheckBox.CheckedChanged += this.CheckBox_CheckedChanged;
            this.ThemeSwitch.Toggled -= this.Switch_Toggled;
            this.ThemeSwitch.IsToggled = false;
            this.ThemeSwitch.Toggled += this.Switch_Toggled;
            this.CurrentThemeLabel.Text = "Dark";
        }
    }

    private void ContentPage_Unloaded(object sender, EventArgs e)
    {
        App.Resumed -= this.OpenPage;
    }

    private void CheckBox_CheckedChanged(object sender, CheckedChangedEventArgs e)
    {
        if(this.SystemThemeCheckBox.IsChecked)
        {
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
            this.ThemeSwitch.IsEnabled = true;
            this.SetTheme();
        }
    }

    private void Switch_Toggled(object sender, ToggledEventArgs e)
    {
        this.SetTheme();
    }


    private void SetTheme()
    {
        if(this.ThemeSwitch.IsToggled)
        {
            Application.Current.UserAppTheme = AppTheme.Light;
            Preferences.Default.Set("Theme", "Light");
            this.CurrentThemeLabel.Text = "Light";
        }
        else
        {
            Application.Current.UserAppTheme = AppTheme.Dark;
            Preferences.Default.Set("Theme", "Dark");
            this.CurrentThemeLabel.Text = "Dark";
        }
    }
}