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
    using hpp::wholebodyStep::createAlignedCOMStabilityConstraint;
    using hpp::core::ConstraintSetPtr_t;
    using hpp::core::ConfigProjectorPtr_t;
    using hpp::model::CenterOfMassComputation;

    namespace impl {
      namespace {
        typedef std::pair <std::string, NumericalConstraintPtr_t>
          NamedConstraint_t;
        typedef std::list <NamedConstraint_t> NamedConstraints_t;

        NamedConstraints_t createSliding (const DevicePtr_t robot,
            const char* prefix, const ConfigurationPtr_t& config,
            const JointPtr_t leftAnkle, const JointPtr_t rightAnkle,
            const CenterOfMassComputationPtr_t comc)
        {
          NamedConstraints_t constraints;
          std::vector <NumericalConstraintPtr_t> numericalConstraints =
            createSlidingStabilityConstraint (robot, comc,
                leftAnkle, rightAnkle, *config);
          std::string p (prefix);
          constraints.push_back (NamedConstraint_t
              (p + std::string ("/relative-com"),
               numericalConstraints [0]));
          constraints.push_back (NamedConstraint_t
              (p + std::string ("/relative-orientation"),
               numericalConstraints [1]));
          constraints.push_back (NamedConstraint_t
              (p + std::string ("/relative-position"),
               numericalConstraints [2]));
          constraints.push_back (NamedConstraint_t
              (p + std::string ("/orientation-left-foot"),
               numericalConstraints [3]));
          constraints.push_back (NamedConstraint_t
              (p + std::string ("/position-left-foot"),
               numericalConstraints [4]));
          return constraints;
        }

        NamedConstraints_t createAlignedCOM (const DevicePtr_t robot,
            const char* prefix, const ConfigurationPtr_t& config,
            const JointPtr_t leftAnkle, const JointPtr_t rightAnkle,
            const CenterOfMassComputationPtr_t comc)
        {
          NamedConstraints_t constraints;
          std::vector <NumericalConstraintPtr_t> numericalConstraints =
            createAlignedCOMStabilityConstraint (robot, comc,
                leftAnkle, rightAnkle, *config);
          std::string p (prefix);
          constraints.push_back (NamedConstraint_t
              (p + std::string ("/com-between-feet"),
               numericalConstraints [0]));
          constraints.push_back (NamedConstraint_t
              (p + std::string ("/orientation-right"),
               numericalConstraints [1]));
          constraints.push_back (NamedConstraint_t
              (p + std::string ("/position-right"),
               numericalConstraints [2]));
          constraints.push_back (NamedConstraint_t
              (p + std::string ("/orientation-left"),
               numericalConstraints [3]));
          constraints.push_back (NamedConstraint_t
              (p + std::string ("/position-left"),
               numericalConstraints [4]));
          return constraints;
        }
      }

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
       const char* leftAnkle, const char* rightAnkle, const char* comName,
       const StaticStabilityType type)
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
          CenterOfMassComputationPtr_t comc;
          if (comN.compare ("") == 0) {
            comc = CenterOfMassComputation::create (robot);
            comc->add (robot->rootJoint ());
            comc->computeMass ();
          } else {
            comc = problemSolver_->centerOfMassComputation (comN);
            if (!comc)
              throw Error ("This CenterOfMassComputation does not exist");
          }
          NamedConstraints_t nc;
          switch ( type ) {
            case hpp::corbaserver::wholebody_step::Problem::SLIDING:
              nc = createSliding (robot, prefix, config, la, ra, comc);
              break;
            case hpp::corbaserver::wholebody_step::Problem::ALIGNED_COM:
              nc = createAlignedCOM (robot, prefix, config, la, ra, comc);
              break;
            default:	
              throw Error ("Unkown StatticStability type.");
              break;
          }
          for (NamedConstraints_t::const_iterator it = nc.begin ();
              it != nc.end (); ++it) {
            problemSolver_->addNumericalConstraint
              (it->first, it->second->functionPtr ());
            problemSolver_->comparisonType
              (it->first, it->second->comparisonType ());
          }
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
