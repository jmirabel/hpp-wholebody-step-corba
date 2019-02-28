// Copyright (C) 2011,2012,2013,2014 CNRS-LAAS
// Author: Florent Lamiraux.
//
// This file is part of the hpp-wholebody-step-corba.
//
// hpp-wholebody-step-corba is free software: you can redistribute
// it and/or modify it under the terms of the GNU Lesser General
// Public License as published by the Free Software Foundation, either
// version 3 of the License, or (at your option) any later version.
//
// hpp-wholebody-step-corba is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with hpp-wholebody-step-corba.  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/util/exception.hh>
#include "hpp/corbaserver/wholebody-step/server.hh"
#include "problem.impl.hh"

namespace hpp {
  namespace wholebodyStep {
    Server::Server (bool multiThread)
      : corbaServer::ServerPlugin (multiThread)
      , impl_ (NULL)
    {}

    Server::~Server () { if (impl_) delete impl_;}

    std::string Server::name () const
    {
      static const std::string n ("hpp-wholebody-step");
      return n;
    }

    /// Start corba server
    void Server::startCorbaServer(const std::string& contextId,
				  const std::string& contextKind)
    {
      impl_ = new corba::Server <impl::Problem> (0, NULL, multithread_, "child");
      impl_->implementation().setServer (this);

      if (impl_->startCorbaServer
          (contextId, contextKind, "wholebodyStep", "problem") != 0) {
	HPP_THROW_EXCEPTION (hpp::Exception, "Failed to start corba server.");
      }
    }
  } // namespace wholebodyStep
} // namespace hpp

HPP_CORBASERVER_DEFINE_PLUGIN(hpp::wholebodyStep::Server)
