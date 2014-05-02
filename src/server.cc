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
    Server::Server (int argc, char *argv[], bool multiThread,
		    const std::string& poaName) : 
      impl_ (new corba::Server <impl::Problem>
	     (argc, argv, multiThread, poaName)) {}
    Server::~Server () { delete impl_;}
    void Server::setProblemSolver (ProblemSolverPtr_t problemSolver)
    {
      impl_->implementation ().setProblemSolver (problemSolver);
    }

    /// Start corba server
    void Server::startCorbaServer(const std::string& contextId,
				  const std::string& contextKind,
				  const std::string& objectId,
				  const std::string& objectKind)
    {
      if (impl_->startCorbaServer(contextId, contextKind, objectId, objectKind)
	  != 0) {
	HPP_THROW_EXCEPTION (hpp::Exception, "Failed to start corba server.");
      }
    }
  } // namespace wholebodyStep
} // namespace hpp
