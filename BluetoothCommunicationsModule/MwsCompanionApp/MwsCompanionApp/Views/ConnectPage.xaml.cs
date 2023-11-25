using MwsCompanionApp.ViewModels;

namespace MwsCompanionApp.Views;

/// <summary>
/// Represents the main page for managing the MWS connection.
/// </summary>
public partial class ConnectPage : ContentPage
{
    /// <summary>
    /// Creates a page for managing the MWS connection.
    /// </summary>
	public ConnectPage(MwsConnectViewModel viewModel)
	{
		this.InitializeComponent();
        this.BindingContext = viewModel;
    }
}