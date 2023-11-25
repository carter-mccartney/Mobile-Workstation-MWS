using MwsCompanionApp.Objects;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Interfaces
{
    /// <summary>
    /// Represents a creator of MWS instances.
    /// </summary>
    public interface IMwsFactory
    {
        /// <summary>
        /// Creates an MWS instance with the given name.
        /// </summary>
        /// <param name="name">
        /// The name of the MWS.
        /// </param>
        /// <returns>
        /// A new MWS object.
        /// </returns>
        Mws Create(string name);
    }
}
