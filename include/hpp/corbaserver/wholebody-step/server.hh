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

#ifndef HPP_WHOLEBODY_STEP_CORBA_SERVER_HH
# define HPP_WHOLEBODY_STEP_CORBA_SERVER_HH

# include <hpp/corba/template/server.hh>
# include <hpp/corbaserver/server-plugin.hh>
# include <hpp/corbaserver/problem-solver-map.hh>
# include <hpp/corbaserver/wholebody-step/fwd.hh>
# include <hpp/corbaserver/wholebody-step/config.hh>

namespace hpp {
  namespace wholebodyStep {
    namespace impl {
      class Problem;
    } // namespace impl
          
    class HPP_WHOLEBODY_STEP_CORBA_DLLAPI Server : public corbaServer::ServerPlugin
    {
    public:
      Server (bool multiThread = false);

      ~Server ();

      /// Start corba server
      /// Call hpp::corba::Server <impl::Problem>::startCorbaServer
      void startCorbaServer(const std::string& contextId,
			    const std::string& contextKind);

      std::string name () const;

    private:
      corba::Server <impl::Problem>* impl_;
    };
  } // namespace wholebodyStep
} // namespace hpp
  
#endif // HPP_WHOLEBODY_STEP_CORBA_SERVER_HH
