using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Services
{
    public partial class PermissionsService
    {
        public partial void CheckPermissions() 
        {
            this.IsBluetoothPermitted = true;
        }
    }
}
