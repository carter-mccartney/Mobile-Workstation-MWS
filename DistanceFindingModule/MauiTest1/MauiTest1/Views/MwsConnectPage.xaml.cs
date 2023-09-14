using MauiTest1.Objects;
using MauiTest1.ViewModels;

namespace MauiTest1.Views;

public partial class MwsConnectPage : ContentPage
{

    private MwsConnectViewModel _viewModel;

	public MwsConnectPage(MwsConnectViewModel viewModel)
	{
		this.InitializeComponent();
        this.BindingContext = this._viewModel = viewModel;
	}

    private void Button_Clicked(object sender, EventArgs e)
    {
        this._viewModel.IsRefreshing = true;
    }

    private async void Refresh(object sender, EventArgs e)
    {
        await this._viewModel.Refresh();
    }

    private void Button_Clicked_1(object sender, EventArgs e)
    {
        MwsConnection connection = (MwsConnection)((BindableObject)sender).BindingContext;
        this._viewModel.Connect(connection);
    }

    private void Button_Clicked_2(object sender, EventArgs e)
    {
        this._viewModel.Disconnect();
    }

    private void ContentPage_Loaded(object sender, EventArgs e)
    {
        this._viewModel.Initialize();
    }
}