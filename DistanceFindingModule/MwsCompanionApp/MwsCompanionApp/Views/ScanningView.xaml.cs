using MwsCompanionApp.Objects;
using MwsCompanionApp.ViewModels;
using System.ComponentModel;
using System.Diagnostics;

namespace MwsCompanionApp.Views;

/// <summary>
/// Represents a view for scanning for MWS units.
/// </summary>
public partial class ScanningView : ContentView
{

    private MwsConnectViewModel _viewModel;

    /// <summary>
    /// Creates a view for scanning for MWS units.
    /// </summary>
	public ScanningView()
	{
		this.InitializeComponent();
    }

    /// <summary>
    /// Invokes the refresh event.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This should be the refresh button.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored in this handler.
    /// </param>
    private void Refresh_Clicked(object sender, EventArgs e)
    {
        this._viewModel.IsRefreshing = true;
    }

    /// <summary>
    /// Refreshes the nearby MWS list.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This should be the refresh view.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored in this handler.
    /// </param>
    private async void Refresh(object sender, EventArgs e)
    {
        await this._viewModel.Refresh();

        // For some reason the itemssource will not update, so doing it manually can work.
        App.Current.Dispatcher.Dispatch(() =>
        {
            this.MwsListView.ItemsSource = null;
            this.MwsListView.ItemsSource = this._viewModel.Services.ScanningService.AvailableConnections;
        });
    }

    /// <summary>
    /// Connects to the current MWS.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This has the MWS as the binding context.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored in this handler.
    /// </param>
    private void Connect_Clicked(object sender, EventArgs e) 
    {
        ((Mws)((BindableObject)sender).BindingContext).Connect();
    }

    /// <summary>
    /// Registers the view-model on load and registers the handler for disconnects.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This should be the view.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored in this handler.
    /// </param>
    private void ContentView_Loaded(object sender, EventArgs e)
    {
        this._viewModel = ((MwsConnectViewModel)this.BindingContext);
        this._viewModel.Services.EventService.BluetoothChanged += this.ClearList;

        // Start loading.
        this._viewModel.IsRefreshing = true;
    }

    /// <summary>
    /// Deregisters handlers before unloading.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event. This should be the view.
    /// </param>
    /// <param name="e">
    /// Context for the event. This is ignored in this handler.
    /// </param>
    private void ContentView_Unoaded(object sender, EventArgs e)
    {
        this._viewModel.Services.EventService.BluetoothChanged -= this.ClearList;
    }

    /// <summary>
    /// Clears the list of available MWS units. Required because of the binding workaround.
    /// </summary>
    /// <param name="sender">
    /// The object invoking the event.
    /// </param>
    /// <param name="e">
    /// Context for the event. If true, bluetooth has connected and if false it has disconnected.
    /// </param>
    private void ClearList(object sender, bool e)
    {
        App.Current.Dispatcher.Dispatch(() =>
        {
            this.MwsListView.ItemsSource = null;
            this.MwsListView.ItemsSource = this._viewModel.Services.ScanningService.AvailableConnections;
        });
    }
}