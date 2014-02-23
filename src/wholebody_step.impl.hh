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

#ifndef HPP_WHOLEBODY_STEP_CORBA_WHOLEBODY_STEP_HH
# define HPP_WHOLEBODY_STEP_CORBA_WHOLEBODY_STEP_HH

# include <omniORB4/CORBA.h>
# include <hpp/core/problem-solver.hh>
# include <hpp/corbaserver/wholebody-step/fwd.hh>
# include "wholebody_step.hh"

namespace hpp {
  namespace wholebodyStep {
    namespace impl {
      class WholebodyStep : public virtual POA_hpp::WholebodyStep
      {
      public:
	WholebodyStep ();
	void setProblemSolver (const ProblemSolverPtr_t& problemSolver);
	virtual CORBA::Short
	addStaticStabilityConstraints (const hpp::dofSeq& dofArray);
	virtual CORBA::Short
	generateGoalConfig (CORBA::Double x, CORBA::Double y, CORBA::Double z,
			    CORBA::UShort nbConfig);
      private:
	core::ProblemSolverPtr_t problemSolver_;
      }; // class WholebodyStep
    } // namespace impl
  } // namespace wholebodyStep
} // namespace hpp

#endif //HPP_WHOLEBODY_STEP_CORBA_WHOLEBODY_STEP_HH
