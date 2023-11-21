using Android.Bluetooth.LE;
using Android.Runtime;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Platforms.Android.Handlers
{
    /// <summary>
    /// A representation of a callback for scanning.
    /// </summary>
    public class GenericScanCallback : ScanCallback
    {
        /// <summary>
        /// The action to take on a scan result.
        /// </summary>
        public Action<ScanCallbackType, ScanResult> OnScanResultCallback { get; set; }

        /// <summary>
        /// The action to take on batch scan results
        /// </summary>
        public Action<IList<ScanResult>> OnBatchScanResultsCallback { get; set; }

        /// <summary>
        /// The action to take on failed scans.
        /// </summary>
        public Action<ScanFailure> OnScanFailedCallback { get; set; }

        /// <inheritdoc/>
        public override void OnScanResult([GeneratedEnum] ScanCallbackType callbackType, ScanResult result)
        {
            base.OnScanResult(callbackType, result);
            App.Current.Dispatcher.Dispatch(() =>
            {
                this.OnScanResultCallback?.Invoke(callbackType, result);
            });
        }

        /// <inheritdoc/>
        public override void OnBatchScanResults(IList<ScanResult> results)
        {
            base.OnBatchScanResults(results);
            App.Current.Dispatcher.Dispatch(() =>
            {
                this.OnBatchScanResultsCallback?.Invoke(results);
            });
        }

        /// <inheritdoc/>
        public override void OnScanFailed([GeneratedEnum] ScanFailure errorCode)
        {
            base.OnScanFailed(errorCode);
            App.Current.Dispatcher.Dispatch(() =>
            {
                this.OnScanFailedCallback?.Invoke(errorCode);
            });
        }
    }
}
