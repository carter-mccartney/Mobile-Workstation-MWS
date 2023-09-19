using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using CoreBluetooth;
using Foundation;
using Plugin.BLE;
namespace MauiTest1.Services
{
    public partial class BackgroundService
    {
        CBPeripheralManager blueToothManager; //= new CBPeripheralManager();
        //used random string found online
        private CBUUID _uuidService = CBUUID.FromString("625D74A2-3E61-4D1E-8949-8BE42DFDC6DA");

        private CBPeripheralManager _PeripheralManager;
        private CBManager _CBManager;

        //private CBUUID _uuidService = CBUUID.FromString("625D74A2-3E61-4D1E-8949-8BE42DFDC6DA");


        private void blueToothManager_ServiceAdded(object sender, CBPeripheralManagerServiceEventArgs e)
        {
            System.Diagnostics.Debug.WriteLine("Service added");
        }
        private void blueToothManager_AdvertisingStarted(object sender, NSErrorEventArgs e)
        {
            System.Diagnostics.Debug.WriteLine("Advertising started");
        }

        private void blueToothManager_StateUpdated(object sender, EventArgs e)
        {
            System.Diagnostics.Debug.WriteLine("State=" + blueToothManager.State);

            if ((long)blueToothManager.State == 5)
            {
                //set up the GATT service
                CBUUID uuidCharact = CBUUID.FromString("5BDAFB34-DA3F-40AA-8F9C-8AD409CB0063");
                NSData readme = NSData.FromString("readme!");
                CBMutableCharacteristic _CharacRead = new CBMutableCharacteristic(uuidCharact, CBCharacteristicProperties.Read, readme, CBAttributePermissions.Readable);
                CBMutableService service = new CBMutableService(_uuidService, true);
                service.Characteristics = new CBMutableCharacteristic[] { _CharacRead };

                blueToothManager.AddService(service);

                StartAdvertisingOptions advData = new StartAdvertisingOptions { LocalName = "my GATT!", ServicesUUID = new CBUUID[] { _uuidService } };
                blueToothManager.StartAdvertising(advData);
            }
        }




















        public partial void Initialize()
        {
            //blueToothManager.Init();
            blueToothManager = new CBPeripheralManager();
            blueToothManager.Init();

            //blueToothManager.StateUpdated += blueToothManager_StateUpdated;
           


        }





        public partial void Start()
        {
            blueToothManager.AdvertisingStarted += blueToothManager_AdvertisingStarted;
            blueToothManager.ServiceAdded += blueToothManager_ServiceAdded;
            blueToothManager.StateUpdated += blueToothManager_StateUpdated;
        }

        

        public partial void Stop()
        {
            blueToothManager.RemoveAllServices();//maybe
        }
    }
   
}
