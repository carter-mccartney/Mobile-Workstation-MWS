using MwsCompanionApp.Interfaces;
using MwsCompanionApp.Objects;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MwsCompanionApp.Services
{
    /// <inheritdoc/>
    public class MwsFactory : IMwsFactory
    {
        /// <summary>
        /// The services for the application.
        /// </summary>
        private ServicesContainer _services;

        /// <summary>
        /// Creates a factory for MWS objects.
        /// </summary>
        /// <param name="services">
        /// The services for the application.
        /// </param>
        public MwsFactory(ServicesContainer services) 
        {
            this._services = services;
        }

        /// <inheritdoc/>
        public Mws Create(string name)
        {
            return new Mws(this._services, name);
        }
    }
}
