using MauiTest1.Services;
using MauiTest1.ViewModels;
using MauiTest1.Views;
using Microsoft.Extensions.Logging;

namespace MauiTest1;

public static class MauiProgram
{
	public static MauiApp CreateMauiApp()
	{
		var builder = MauiApp.CreateBuilder();
        builder.UseMauiApp<App>()
               .RegisterViews()
               .RegisterViewModels()
               .RegisterServices()
               .RegisterPlatformServices()
			   .ConfigureFonts(fonts =>
			   {
				   fonts.AddFont("OpenSans-Regular.ttf", "OpenSansRegular");
				   fonts.AddFont("OpenSans-Semibold.ttf", "OpenSansSemibold");
			   });

#if DEBUG
		builder.Logging.AddDebug();
#endif

		return builder.Build();
	}
}
