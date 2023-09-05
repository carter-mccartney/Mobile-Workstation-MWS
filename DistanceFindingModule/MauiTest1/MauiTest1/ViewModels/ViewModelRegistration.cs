using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MauiTest1.ViewModels
{
    public static class ViewModelRegistration
    {
        public static MauiAppBuilder RegisterViewModels(this MauiAppBuilder builder) 
        {
            builder.Services.AddTransient(typeof(MwsConnectViewModel));
            builder.Services.AddTransient(typeof(MwsConfigurationViewModel));
            return builder;
        }
    }
}
