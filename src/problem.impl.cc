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
#include <hpp/model/center-of-mass-computation.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/wholebody-step/static-stability-constraint.hh>
#include "problem.impl.hh"

namespace hpp {
  namespace wholebodyStep {
    using hpp::wholebodyStep::createSlidingStabilityConstraint;
    using hpp::wholebodyStep::createStabilityConstraint;
    using hpp::core::ConstraintSetPtr_t;
    using hpp::core::ConfigProjectorPtr_t;
    namespace impl {

      static ConfigurationPtr_t dofSeqToConfig
      (ProblemSolverPtr_t problemSolver, const hpp::dofSeq& dofArray)
      {
	size_type configDim = (size_type) dofArray.length();
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
	for (size_type iDof=0; iDof < configDim; ++iDof) {
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
      (const char* prefix, const hpp::dofSeq& dofArray,
       const char* leftAnkle, const char* rightAnkle, const char* comName)
      throw (hpp::Error)
      {
        using model::CenterOfMassComputationPtr_t;
	using core::DifferentiableFunctionPtr_t;
	try {
	  ConfigurationPtr_t config = dofSeqToConfig (problemSolver_, dofArray);
	  const DevicePtr_t& robot (problemSolver_->robot ());
	  if (!robot) {
	    throw Error ("You should set the robot before defining"
			 " constraints.");
	  }
	  JointPtr_t la = robot->getJointByName (leftAnkle);
	  JointPtr_t ra = robot->getJointByName (rightAnkle);
          std::string comN (comName);
	  std::vector <DifferentiableFunctionPtr_t> numericalConstraints;
          if (comN.compare ("") == 0)
            numericalConstraints =
              createSlidingStabilityConstraint (robot, la, ra, *config);
          else {
            CenterOfMassComputationPtr_t comc =
              problemSolver_->centerOfMassComputation (comN);
            if (!comc)
              throw Error ("This CenterOfMassComputation does not exist");
            numericalConstraints = createSlidingStabilityConstraint (robot, 
                  comc, la, ra, *config);
          }
	  std::string p (prefix);
	  problemSolver_->addNumericalConstraint
	    (p + std::string ("/relative-com"), numericalConstraints [0]);
	  problemSolver_->addNumericalConstraint
	    (p + std::string ("/relative-orientation"),
	     numericalConstraints [1]);
	  problemSolver_->addNumericalConstraint
	     (p + std::string ("/relative-position"), numericalConstraints [2]);
	  problemSolver_->addNumericalConstraint
	      (p + std::string ("/orientation-left-foot"),
	       numericalConstraints [3]);
	  problemSolver_->addNumericalConstraint
	       (p + std::string ("/position-left-foot"),
		numericalConstraints [4]);
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Problem::addStabilityConstraints
      (const char* prefix, const hpp::dofSeq& dofArray,
       const char* leftAnkle, const char* rightAnkle, const char* comName)
      throw (hpp::Error)
      {
        using model::CenterOfMassComputationPtr_t;
	using core::DifferentiableFunctionPtr_t;
	try {
	  ConfigurationPtr_t config = dofSeqToConfig (problemSolver_, dofArray);
	  const DevicePtr_t& robot (problemSolver_->robot ());
	  if (!robot) {
	    throw Error ("You should set the robot before defining"
			 " constraints.");
	  }
	  JointPtr_t la = robot->getJointByName (leftAnkle);
	  JointPtr_t ra = robot->getJointByName (rightAnkle);
          std::string comN (comName);
	  std::vector <core::NumericalConstraintPtr_t> numericalConstraints;
          if (comN.compare ("") == 0) {
            model::CenterOfMassComputationPtr_t comc = 
              model::CenterOfMassComputation::create (robot);
            comc->add (robot->rootJoint ());
            comc->computeMass ();
            numericalConstraints =
              createStabilityConstraint (robot, comc, la, ra, *config);
          } else {
            CenterOfMassComputationPtr_t comc =
              problemSolver_->centerOfMassComputation (comN);
            if (!comc)
              throw Error ("This CenterOfMassComputation does not exist");
            numericalConstraints = createStabilityConstraint (robot, 
                  comc, la, ra, *config);
          }
	  std::string p (prefix);
	  problemSolver_->addNumericalConstraint
	    (p + std::string ("/com-between-feet"), numericalConstraints [0]->functionPtr ());
	  problemSolver_->comparisonType
	    (p + std::string ("/com-between-feet"), numericalConstraints [0]->comparisonType ());
	  problemSolver_->addNumericalConstraint
	     (p + std::string ("/orientation-right"), numericalConstraints [1]->functionPtr ());
	  problemSolver_->comparisonType
	     (p + std::string ("/orientation-right"), numericalConstraints [1]->comparisonType ());
	  problemSolver_->addNumericalConstraint
	    (p + std::string ("/orientation-left"), numericalConstraints [2]->functionPtr ());
	  problemSolver_->comparisonType
	    (p + std::string ("/orientation-left"), numericalConstraints [2]->comparisonType ());
	  problemSolver_->addNumericalConstraint
	      (p + std::string ("/position-right"), numericalConstraints [3]->functionPtr ());
	  problemSolver_->comparisonType
	      (p + std::string ("/position-right"), numericalConstraints [3]->comparisonType ());
	  problemSolver_->addNumericalConstraint
	       (p + std::string ("/position-left"), numericalConstraints [4]->functionPtr ());
	  problemSolver_->comparisonType
	       (p + std::string ("/position-left"), numericalConstraints [4]->comparisonType ());
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Problem::generateGoalConfig
      (CORBA::Double, CORBA::Double, CORBA::Double,
       CORBA::UShort) throw (hpp::Error)
      {
	assert (problemSolver_);
      }
    } // namespace impl
  } // namespace wholebodyStep
} // namespace hpp
