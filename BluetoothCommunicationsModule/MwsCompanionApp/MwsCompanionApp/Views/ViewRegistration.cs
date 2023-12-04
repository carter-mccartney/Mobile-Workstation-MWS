using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Views
{
    /// <summary>
    /// Contains an extension method for the app builder to register views.
    /// </summary>
    public static class ViewRegistration
    {
        /// <summary>
        /// Registers the views in the program.
        /// </summary>
        /// <param name="builder">
        /// The app builder for the program.
        /// </param>
        /// <returns>
        /// The app builder for the program.
        /// </returns>
        public static MauiAppBuilder RegisterViews(this MauiAppBuilder builder)
        {
            builder.Services.AddTransient(typeof(ConnectPage));
            builder.Services.AddTransient(typeof(ScanningView));
            builder.Services.AddTransient(typeof(NoConnectionView));
            builder.Services.AddTransient(typeof(SettingsPage));
            return builder;
        }
    }
}
