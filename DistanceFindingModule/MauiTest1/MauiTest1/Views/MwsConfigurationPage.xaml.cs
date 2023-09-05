using MauiTest1.ViewModels;

namespace MauiTest1.Views;

public partial class MwsConfigurationPage : ContentPage
{

    private MwsConfigurationViewModel _viewModel;

	public MwsConfigurationPage(MwsConfigurationViewModel viewModel)
	{
		this.InitializeComponent();
        this.BindingContext = this._viewModel = viewModel;
	}
}