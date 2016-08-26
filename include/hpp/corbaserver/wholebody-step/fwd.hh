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

#ifndef HPP_WHOLEBODY_STEP_CORBA_FWD_HH
# define HPP_WHOLEBODY_STEP_CORBA_FWD_HH

# include <hpp/core/fwd.hh>

namespace hpp {
  namespace wholebodyStep {
    typedef core::ProblemSolver ProblemSolver;
    typedef core::ProblemSolverPtr_t ProblemSolverPtr_t;
    typedef core::ConfigProjectorPtr_t ConfigProjectorPtr_t;
    typedef core::Configuration_t Configuration_t;
    typedef core::ConfigurationPtr_t ConfigurationPtr_t;
    typedef pinocchio::DevicePtr_t DevicePtr_t;
    typedef pinocchio::HumanoidRobotPtr_t HumanoidRobotPtr_t;
    typedef pinocchio::HumanoidRobot HumanoidRobot;

    class Server;
  } // namespace wholebodyStep
} // namespace hpp
  
#endif // HPP_WHOLEBODY_STEP_CORBA_SERVER_HH
