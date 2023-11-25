using MwsCompanionApp.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Services
{
    /// <summary>
    /// Contains an extension method for the app builder to register services.
    /// </summary>
    public static class ServiceRegistration
    {
        /// <summary>
        /// Registers the services in the program.
        /// </summary>
        /// <param name="builder">
        /// The app builder for the program.
        /// </param>
        /// <returns>
        /// The app builder for the program.
        /// </returns>
        public static MauiAppBuilder RegisterServices(this MauiAppBuilder builder)
        {
            builder.Services.AddSingleton(typeof(IConnectionManagementService), typeof(ConnectionManagementService));
            builder.Services.AddSingleton(typeof(IScanningService), typeof(ScanningService));
            builder.Services.AddSingleton(typeof(IEventService), typeof(EventService));
            builder.Services.AddSingleton(typeof(IMwsFactory), typeof(MwsFactory));
            builder.Services.AddSingleton(typeof(IPermissionsService), typeof(PermissionsService));
            builder.Services.AddSingleton(typeof(ServicesContainer));
            return builder;
        }
    }
}
