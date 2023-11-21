using Android.Bluetooth;
using Android.Bluetooth.LE;
using Android.Runtime;
using Android.Systems;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Platforms.Android.Handlers
{
    /// <summary>
    /// Represents a set of callbacks for the connection to a GATT server.
    /// </summary>
    public class GenericBluetoothGattCallback : BluetoothGattCallback
    {
        /// <summary>
        /// The callback for the conncetion state changed handler.
        /// </summary>
        public Action<BluetoothGatt, GattStatus, ProfileState> OnConnectionStateChangedCallback { get; set; }

        /// <summary>
        /// The callback for the characteristic changed handler on Android version 33+.
        /// </summary>
        public Action<BluetoothGatt, BluetoothGattCharacteristic, byte[]> OnCharacteristicChangedCallback { get; set; }

        /// <summary>
        /// The callback for the characteristic changed handler on legacy Android versions.
        /// </summary>
        public Action<BluetoothGatt, BluetoothGattCharacteristic> OnCharacteristicChangedLegacyCallback { get; set; }

        /// <summary>
        /// The callback for the characteristic read handler on Android version 33+.
        /// </summary>
        public Action<BluetoothGatt, BluetoothGattCharacteristic, byte[], GattStatus> OnCharacteristicReadCallback { get; set; }

        /// <summary>
        /// The callback for the characteristic read handler on legacy Android versions.
        /// </summary>
        public Action<BluetoothGatt, BluetoothGattCharacteristic, GattStatus> OnCharacteristicReadLegacyCallback { get; set; }

        /// <summary>
        /// The callback for the characteristic write handler.
        /// </summary>
        public Action<BluetoothGatt, BluetoothGattCharacteristic, GattStatus> OnCharacteristicWriteCallback { get; set; }

        /// <summary>
        /// The callback for the descriptor read handler on Android version 33+.
        /// </summary>
        public Action<BluetoothGatt, BluetoothGattDescriptor, GattStatus, byte[]> OnDescriptorReadCallback { get; set; }

        /// <summary>
        /// The callback for the descriptor read handler on legacy Android versions.
        /// </summary>
        public Action<BluetoothGatt, BluetoothGattDescriptor, GattStatus> OnDescriptorReadLegacyCallback { get; set; }

        /// <summary>
        /// The callback for the descriptor write handler.
        /// </summary>
        public Action<BluetoothGatt, BluetoothGattDescriptor, GattStatus> OnDescriptorWriteCallback { get; set; }

        /// <summary>
        /// The callback for the descriptor changed handler.
        /// </summary>
        public Action<BluetoothGatt, int, GattStatus> OnMtuChangedCallback { get; set; }

        /// <summary>
        /// The callback for the PHY read handler.
        /// </summary>
        public Action<BluetoothGatt, ScanSettingsPhy, ScanSettingsPhy, GattStatus> OnPhyReadCallback { get; set; }

        /// <summary>
        /// The callback for the PHY updated handler.
        /// </summary>
        public Action<BluetoothGatt, ScanSettingsPhy, ScanSettingsPhy, GattStatus> OnPhyUpdatedCallback { get; set; }

        /// <summary>
        /// The callback for the read remote rssi handler.
        /// </summary>
        public Action<BluetoothGatt, int, GattStatus> OnReadRemoteRssiCallback { get; set; }

        /// <summary>
        /// The callback for the reliable write completed handler.
        /// </summary>
        public Action<BluetoothGatt, GattStatus> OnReliableWriteCompletedCallback { get; set; }

        /// <summary>
        /// The callback for the service changed handler.
        /// </summary>
        public Action<BluetoothGatt> OnServiceChangedCallback { get; set; }

        /// <summary>
        /// The callback for the services discovered handler.
        /// </summary>
        public Action<BluetoothGatt, GattStatus> OnServicesDiscoveredCallback { get; set; }

        /// <inheritdoc/>
        public override void OnConnectionStateChange(BluetoothGatt gatt, [GeneratedEnum] GattStatus status, [GeneratedEnum] ProfileState newState)
        {
            base.OnConnectionStateChange(gatt, status, newState);
            this.OnConnectionStateChangedCallback?.Invoke(gatt, status, newState);
        }

        /// <inheritdoc/>
        public override void OnCharacteristicChanged(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, byte[] value)
        {
            if(OperatingSystem.IsAndroidVersionAtLeast(33))
            {
                base.OnCharacteristicChanged(gatt, characteristic, value);
                this.OnCharacteristicChangedCallback?.Invoke(gatt, characteristic, value);
            }
        }

        /// <inheritdoc/>
        public override void OnCharacteristicChanged(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic)
        {
            if(!OperatingSystem.IsAndroidVersionAtLeast(33))
            {
                base.OnCharacteristicChanged(gatt, characteristic);
                this.OnCharacteristicChangedLegacyCallback?.Invoke(gatt, characteristic);
            }
        }

        /// <inheritdoc/>
        public override void OnCharacteristicRead(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, byte[] value, [GeneratedEnum] GattStatus status)
        {
            App.Current.Dispatcher.Dispatch(() =>
            {
                if(OperatingSystem.IsAndroidVersionAtLeast(33))
                {
                    base.OnCharacteristicRead(gatt, characteristic, value, status);
                    this.OnCharacteristicReadCallback?.Invoke(gatt, characteristic, value, status);
                }
            });
        }

        /// <inheritdoc/>
        public override void OnCharacteristicRead(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, [GeneratedEnum] GattStatus status)
        {
            App.Current.Dispatcher.Dispatch(() =>
            {
                if(!OperatingSystem.IsAndroidVersionAtLeast(33))
                {
                    base.OnCharacteristicRead(gatt, characteristic, status);
                    this.OnCharacteristicReadLegacyCallback?.Invoke(gatt, characteristic, status);
                }
            });
        }

        /// <inheritdoc/>
        public override void OnCharacteristicWrite(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, [GeneratedEnum] GattStatus status)
        {
            base.OnCharacteristicWrite(gatt, characteristic, status);
            this.OnCharacteristicWriteCallback(gatt, characteristic, status);
        }

        /// <inheritdoc/>
        public override void OnDescriptorRead(BluetoothGatt gatt, BluetoothGattDescriptor descriptor, [GeneratedEnum] GattStatus status, byte[] value)
        {
            if(OperatingSystem.IsAndroidVersionAtLeast(33))
            {
                base.OnDescriptorRead(gatt, descriptor, status, value);
                this.OnDescriptorReadCallback?.Invoke(gatt, descriptor, status, value);
            }
        }

        /// <inheritdoc/>
        public override void OnDescriptorRead(BluetoothGatt gatt, BluetoothGattDescriptor descriptor, [GeneratedEnum] GattStatus status)
        {
            if(!OperatingSystem.IsAndroidVersionAtLeast(33))
            {
                base.OnDescriptorRead(gatt, descriptor, status);
                this.OnDescriptorReadLegacyCallback?.Invoke(gatt, descriptor, status);
            }
        }

        /// <inheritdoc/>
        public override void OnDescriptorWrite(BluetoothGatt gatt, BluetoothGattDescriptor descriptor, [GeneratedEnum] GattStatus status)
        {
            base.OnDescriptorWrite(gatt, descriptor, status);
            this.OnDescriptorWriteCallback?.Invoke(gatt, descriptor, status);
        }

        /// <inheritdoc/>
        public override void OnMtuChanged(BluetoothGatt gatt, int mtu, [GeneratedEnum] GattStatus status)
        {
            base.OnMtuChanged(gatt, mtu, status);
            this.OnMtuChangedCallback?.Invoke(gatt, mtu, status);
        }

        /// <inheritdoc/>
        public override void OnPhyRead(BluetoothGatt gatt, [GeneratedEnum] ScanSettingsPhy txPhy, [GeneratedEnum] ScanSettingsPhy rxPhy, [GeneratedEnum] GattStatus status)
        {
            base.OnPhyRead(gatt, txPhy, rxPhy, status);
            this.OnPhyReadCallback?.Invoke(gatt, txPhy, rxPhy, status);
        }

        /// <inheritdoc/>
        public override void OnPhyUpdate(BluetoothGatt gatt, [GeneratedEnum] ScanSettingsPhy txPhy, [GeneratedEnum] ScanSettingsPhy rxPhy, [GeneratedEnum] GattStatus status)
        {
            base.OnPhyUpdate(gatt, txPhy, rxPhy, status);
            this.OnPhyUpdatedCallback?.Invoke(gatt, txPhy, rxPhy, status);
        }

        /// <inheritdoc/>
        public override void OnReadRemoteRssi(BluetoothGatt gatt, int rssi, [GeneratedEnum] GattStatus status)
        {
            base.OnReadRemoteRssi(gatt, rssi, status);
            this.OnReadRemoteRssiCallback?.Invoke(gatt, rssi, status);
        }

        /// <inheritdoc/>
        public override void OnReliableWriteCompleted(BluetoothGatt gatt, [GeneratedEnum] GattStatus status)
        {
            base.OnReliableWriteCompleted(gatt, status);
            this.OnReliableWriteCompletedCallback?.Invoke(gatt, status);
        }

        /// <inheritdoc/>
        public override void OnServiceChanged(BluetoothGatt gatt)
        {
            base.OnServiceChanged(gatt);
            this.OnServiceChangedCallback?.Invoke(gatt);
        }

        /// <inheritdoc/>
        public override void OnServicesDiscovered(BluetoothGatt gatt, [GeneratedEnum] GattStatus status)
        {
            base.OnServicesDiscovered(gatt, status);
            this.OnServicesDiscoveredCallback?.Invoke(gatt, status);
        }
    }
}
