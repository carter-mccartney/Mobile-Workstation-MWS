using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Services
{
    /// <summary>
    /// Contains an extension method for the app builder to register platform services.
    /// </summary>
    public static partial class PlatformRegistration
    {
        /// <summary>
        /// Registers the platform-specific entities in the program.
        /// </summary>
        /// <param name="builder">
        /// The app builder for the program.
        /// </param>
        /// <returns>
        /// The app builder for the program.
        /// </returns>
        public static partial MauiAppBuilder RegisterPlatformServices(this MauiAppBuilder builder);
    }
}
