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

#include <cassert>
#include <hpp/util/debug.hh>
#include <hpp/model/humanoid-robot.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/wholebody-step/static-stability-constraint.hh>
#include "problem.impl.hh"

namespace hpp {
  namespace wholebodyStep {
    using hpp::wholebodyStep::addSlidingStabilityConstraint;
    using hpp::core::ConstraintSetPtr_t;
    using hpp::core::ConfigProjectorPtr_t;
    namespace impl {

      static ConfigurationPtr_t dofSeqToConfig
      (ProblemSolverPtr_t problemSolver, const hpp::dofSeq& dofArray)
      {
	unsigned int configDim = (unsigned int)dofArray.length();
	ConfigurationPtr_t config (new Configuration_t (configDim));
	
	// Get robot in hppPlanner object.
	DevicePtr_t robot = problemSolver->robot ();

	// Compare size of input array with number of degrees of freedom of
	// robot.
	if (configDim != robot->configSize ()) {
	  hppDout (error, "robot configSize (" << robot->configSize ()
		   << ") is different from config size ("
		   << configDim << ")");
	  throw std::runtime_error
	    ("robot nb dof is different from config size");
	}

	// Fill dof vector with dof array.
	for (unsigned int iDof=0; iDof < configDim; ++iDof) {
	  (*config) [iDof] = dofArray [iDof];
	}
	return config;
      }

      Problem::Problem () : problemSolver_ (0x0) {}

      void Problem::setProblemSolver
      (const ProblemSolverPtr_t& problemSolver)
      {
	problemSolver_ = problemSolver;
      }

      void Problem::addStaticStabilityConstraints
      (const char* constraintName, const hpp::dofSeq& dofArray,
       const char* leftAnkle, const char* rightAnkle) throw (hpp::Error)
      {
	try {
	  ConfigurationPtr_t config = dofSeqToConfig (problemSolver_, dofArray);
	  const ConstraintSetPtr_t& constraints
	    (problemSolver_->constraints ());
	  const DevicePtr_t& robot (problemSolver_->robot ());
	  if (!robot) {
	    throw Error ("You should set the robot before defining"
			 " constraints.");
	  }
	  ConfigProjectorPtr_t  configProjector =
	    constraints->configProjector ();
	  if (!configProjector) {
	    configProjector = ConfigProjector::create
	      (robot, constraintName, problemSolver_->errorThreshold (),
	       problemSolver_->maxIterations ());
	    constraints->addConstraint (configProjector);
	  }
	  JointPtr_t la = robot->getJointByName (leftAnkle);
	  JointPtr_t ra = robot->getJointByName (rightAnkle);
	  addSlidingStabilityConstraint (configProjector, robot, la, ra,
					 *config);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Problem::generateGoalConfig
      (CORBA::Double x, CORBA::Double y, CORBA::Double z,
       CORBA::UShort nbConfig) throw (hpp::Error)
      {
	assert (problemSolver_);
      }
    } // namespace impl
  } // namespace wholebodyStep
} // namespace hpp
