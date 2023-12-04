using MwsCompanionApp.Objects;

namespace MwsCompanionApp.Views;

/// <summary>
/// Represents an active connection to an MWS unit.
/// </summary>
public partial class ConnectionView : ContentView
{
    /// <summary>
    /// The currently-bound MWS.
    /// </summary>
    private Mws _viewModel;

    /// <summary>
    /// Creates a view for managing an active connection to an MWS unit.
    /// </summary>
	public ConnectionView()
	{
		this.InitializeComponent();
    }

    /// <summary>
    /// Sets the tab to the follower tab on load.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This should be the view.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored by this handler.
    /// </param>
    private void ContentView_Loaded(object sender, EventArgs e)
    {
        this.TabView.SelectedIndex = 0;
    }

    /// <summary>
    /// Resets the view when a different MWS is in it.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This should be the view.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored by this handler.
    /// </param>
    private void ContentView_BindingContextChanged(object sender, EventArgs e)
    {
        this._viewModel = (Mws)this.BindingContext;
        this.TabView.SelectedIndex = 0;
    }

    /// <summary>
    /// Disconnects from the current connection.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This should be the connection view.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored in this handler.
    /// </param>
    private void Disconnect_Click(object sender, EventArgs e) 
    {
        this._viewModel.Disconnect();
    }

    /// <summary>
    /// Places the MWS in follower mode.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This should be the follow button.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored by this handler.
    /// </param>
    private void FollowButton_Clicked(object sender, EventArgs e)
    {
        this._viewModel.BeginFollowing();
    }

    /// <summary>
    /// Takes the MWS out of follower mode.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This should be the stop following button.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored by this handler.
    /// </param>
    private void StopFollowingButton_Clicked(object sender, EventArgs e)
    {
        this._viewModel.StopFollowing();
    }

    /// <summary>
    /// Calibrates the first one.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This should be the calibrate one button.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored by this handler.
    /// </param>
    private void Calibrate1_Clicked(object sender, EventArgs e) 
    {
        this._viewModel.Calibrate1();
    }

    /// <summary>
    /// Calibrates the second one.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This should be the calibrate two button.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored by this handler.
    /// </param>
    private void Calibrate2_Clicked(object sender, EventArgs e)
    {
        this._viewModel.Calibrate2();
    }

    /// <summary>
    /// Calibrates the third one.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This should be the calibrate three button.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored by this handler.
    /// </param>
    private void Calibrate3_Clicked(object sender, EventArgs e)
    {
        this._viewModel.Calibrate3();
    }

    /// <summary>
    /// Calibrates the fourth one.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This should be the calibrate four button.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored by this handler.
    /// </param>
    private void Calibrate4_Clicked(object sender, EventArgs e)
    {
        this._viewModel.Calibrate4();
    }

    /// <summary>
    /// Changes the range of the MWS.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This should be the range slider.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored by this handler.
    /// </param>
    private void Range_Updated(object sender, EventArgs e) 
    {
        this._viewModel.UpdateRange();
    }
}