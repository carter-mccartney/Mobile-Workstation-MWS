using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.ViewModels
{
    /// <summary>
    /// Contains an extension method for the app builder to register view-models.
    /// </summary>
    public static class ViewModelRegistration
    {
        /// <summary>
        /// Registers the view-models in the program.
        /// </summary>
        /// <param name="builder">
        /// The app builder for the program.
        /// </param>
        /// <returns>
        /// The app builder for the program.
        /// </returns>
        public static MauiAppBuilder RegisterViewModels(this MauiAppBuilder builder)
        {
            builder.Services.AddSingleton(typeof(MwsConnectViewModel));
            return builder;
        }
    }
}
