using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MauiTest1.Services
{
    public static partial class PlatformRegistration
    {
        public static partial MauiAppBuilder RegisterPlatformServices(this MauiAppBuilder builder) 
        {
            return builder;
        }
    }
}
