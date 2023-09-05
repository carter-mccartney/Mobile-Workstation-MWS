using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MauiTest1.Views
{
    public static class ViewRegistration
    {
        public static MauiAppBuilder RegisterViews(this MauiAppBuilder builder)
        {
            builder.Services.AddTransient(typeof(MwsConnectPage));
            builder.Services.AddTransient(typeof(MwsConfigurationPage));
            builder.Services.AddTransient(typeof(SettingsPage));
            return builder;
        }
    }
}
