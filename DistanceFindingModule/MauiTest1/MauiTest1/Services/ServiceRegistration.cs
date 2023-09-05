using MauiTest1.Interfaces;
using Microsoft.Extensions.DependencyInjection;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MauiTest1.Services
{
    public static class ServiceRegistration
    {
        public static MauiAppBuilder RegisterServices(this MauiAppBuilder builder)
        {
            builder.Services.AddSingleton(typeof(IMwsConnectionService), typeof(MwsConnectionService));
            builder.Services.AddSingleton(typeof(IBackgroundService), typeof(BackgroundService));
            return builder;
        }
    }
}
