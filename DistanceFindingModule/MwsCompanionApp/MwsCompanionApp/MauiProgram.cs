using MwsCompanionApp.Services;
using MwsCompanionApp.ViewModels;
using MwsCompanionApp.Views;
using Microsoft.Extensions.Logging;
using Sharpnado.Tabs;
using CommunityToolkit.Maui;
using Plugin.LocalNotification;

namespace MwsCompanionApp;

public static class MauiProgram
{
	public static MauiApp CreateMauiApp()
	{
		var builder = MauiApp.CreateBuilder();
        builder.UseMauiApp<App>()
               .UseMauiCommunityToolkit()
               .RegisterLazyResolution()
               .RegisterViews()
               .RegisterViewModels()
               .RegisterServices()
               .RegisterPlatformServices()
               .UseSharpnadoTabs(loggerEnable: false)
               .UseLocalNotification()
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

/// <summary>
/// Contains methods for extending the app builder to register lazy resolution.
/// </summary>
internal static class MauiAppBuilderLazyExtensions
{
    /// <summary>
    /// Registers the resulting type for lazy dependencies.
    /// </summary>
    /// <param name="builder">
    /// The builder that needs to register the services.
    /// </param>
    /// <returns>
    /// The builder that needs to register the services.
    /// </returns>
    public static MauiAppBuilder RegisterLazyResolution(this MauiAppBuilder builder)
    {
        builder.Services.AddTransient(typeof(Lazy<>), typeof(LazilyResolved<>));
        return builder;
    }
}

/// <summary>
/// Represents a class that may be lazily resolved.
/// </summary>
/// <typeparam name="T">
/// A type to be resolved lazily.
/// </typeparam>
internal class LazilyResolved<T> : Lazy<T>
{
    /// <summary>
    /// Creates an instance of a lazily resolved instance that specifies to get the required service from 
    /// the service provider when first evaluated and no sooner.
    /// </summary>
    /// <param name="serviceProvider"></param>
    public LazilyResolved(IServiceProvider serviceProvider) : base(serviceProvider.GetRequiredService<T>) { }
}
