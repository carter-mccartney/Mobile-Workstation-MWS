using Android.Bluetooth.LE;
using Android.Runtime;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Platforms.Android.Handlers
{
    public class GenericAdvertisingSetCallback : AdvertisingSetCallback
    {
        /// <summary>
        /// The callback for the advertising data set handler.
        /// </summary>
        public Action<AdvertisingSet, AdvertiseResult> OnAdvertisingDataSetCallback { get; set; }

        /// <summary>
        /// The callback for the advertising enabled handler.
        /// </summary>
        public Action<AdvertisingSet, bool ,AdvertiseResult> OnAdvertisingEnabledCallback { get; set; }

        /// <summary>
        /// The callback for the advertising parameters updated handler.
        /// </summary>
        public Action<AdvertisingSet, int, AdvertiseResult> OnAdvertisingParametersUpdatedCallback { get; set; }

        /// <summary>
        /// The callback for the advertising set started handler.
        /// </summary>
        public Action<AdvertisingSet, int, AdvertiseResult> OnAdvertisingSetStartedCallback { get; set; }

        /// <summary>
        /// The callback for the advertising set stopped handler.
        /// </summary>
        public Action<AdvertisingSet> OnAdvertisingSetStoppedCallback { get; set; }

        /// <summary>
        /// The callback for the periodic advertising data set handler.
        /// </summary>
        public Action<AdvertisingSet, AdvertiseResult> OnPeriodicAdvertisingDataSetCallback { get; set; }

        /// <summary>
        /// The callback for the periodic advertsing data set enabled handler.
        /// </summary>
        public Action<AdvertisingSet, bool, AdvertiseResult> OnPeriodicAdvertisingEnabledCallback { get; set; }

        /// <summary>
        /// The callback for the periodic advertising parameters updated handler.
        /// </summary>
        public Action<AdvertisingSet, AdvertiseResult> OnPeriodicAdvertisingParametersUpdatedCallback { get; set; }

        /// <summary>
        /// The callback for the scan response data set callback.
        /// </summary>
        public Action<AdvertisingSet, AdvertiseResult> OnScanResponseDataSetCallback { get; set; }

        /// <inheritdoc/>
        public override void OnAdvertisingDataSet(AdvertisingSet advertisingSet, [GeneratedEnum] AdvertiseResult status)
        {
            base.OnAdvertisingDataSet(advertisingSet, status);
            this.OnAdvertisingDataSetCallback?.Invoke(advertisingSet, status);
        }

        /// <inheritdoc/>
        public override void OnAdvertisingEnabled(AdvertisingSet advertisingSet, bool enable, [GeneratedEnum] AdvertiseResult status)
        {
            base.OnAdvertisingEnabled(advertisingSet, enable, status);
            this.OnAdvertisingEnabledCallback?.Invoke(advertisingSet, enable, status);
        }

        /// <inheritdoc/>
        public override void OnAdvertisingParametersUpdated(AdvertisingSet advertisingSet, int txPower, [GeneratedEnum] AdvertiseResult status)
        {
            base.OnAdvertisingParametersUpdated(advertisingSet, txPower, status);
            this.OnAdvertisingParametersUpdatedCallback?.Invoke(advertisingSet, txPower, status);
        }

        /// <inheritdoc/>
        public override void OnAdvertisingSetStarted(AdvertisingSet advertisingSet, int txPower, [GeneratedEnum] AdvertiseResult status)
        {
            base.OnAdvertisingSetStarted(advertisingSet, txPower, status);
            this.OnAdvertisingSetStartedCallback?.Invoke(advertisingSet, txPower, status);
        }

        /// <inheritdoc/>
        public override void OnAdvertisingSetStopped(AdvertisingSet advertisingSet)
        {
            base.OnAdvertisingSetStopped(advertisingSet);
            this.OnAdvertisingSetStoppedCallback?.Invoke(advertisingSet);
        }

        /// <inheritdoc/>
        public override void OnPeriodicAdvertisingDataSet(AdvertisingSet advertisingSet, [GeneratedEnum] AdvertiseResult status)
        {
            base.OnPeriodicAdvertisingDataSet(advertisingSet, status);
            this.OnPeriodicAdvertisingDataSetCallback?.Invoke(advertisingSet, status);
        }

        /// <inheritdoc/>
        public override void OnPeriodicAdvertisingEnabled(AdvertisingSet advertisingSet, bool enable, [GeneratedEnum] AdvertiseResult status)
        {
            base.OnPeriodicAdvertisingEnabled(advertisingSet, enable, status);
            this.OnPeriodicAdvertisingEnabledCallback?.Invoke(advertisingSet, enable, status);
        }

        /// <inheritdoc/>
        public override void OnPeriodicAdvertisingParametersUpdated(AdvertisingSet advertisingSet, [GeneratedEnum] AdvertiseResult status)
        {
            base.OnPeriodicAdvertisingParametersUpdated(advertisingSet, status);
            this.OnPeriodicAdvertisingParametersUpdatedCallback?.Invoke(advertisingSet, status);
        }

        /// <inheritdoc/>
        public override void OnScanResponseDataSet(AdvertisingSet advertisingSet, [GeneratedEnum] AdvertiseResult status)
        {
            base.OnScanResponseDataSet(advertisingSet, status);
            this.OnScanResponseDataSetCallback?.Invoke(advertisingSet, status);
        }
    }
}
