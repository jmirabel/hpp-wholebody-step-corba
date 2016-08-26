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

#include "problem.impl.hh"

#include <cassert>
#include <hpp/util/debug.hh>
#include <hpp/pinocchio/center-of-mass-computation.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/wholebody-step/static-stability-constraint.hh>

#include <hpp/corbaserver/wholebody-step/server.hh>

namespace hpp {
  namespace wholebodyStep {
    using hpp::wholebodyStep::createSlidingStabilityConstraint;
    using hpp::wholebodyStep::createAlignedCOMStabilityConstraint;
    using hpp::core::ConstraintSetPtr_t;
    using hpp::core::ConfigProjectorPtr_t;
    using hpp::pinocchio::CenterOfMassComputation;

    namespace impl {
      namespace {
        typedef std::pair <std::string, NumericalConstraintPtr_t>
          NamedConstraint_t;
        typedef std::list <NamedConstraint_t> NamedConstraints_t;

        NamedConstraints_t createFixed (const DevicePtr_t robot,
            const char* prefix, const ConfigurationPtr_t& config,
            const JointPtr_t leftAnkle, const JointPtr_t rightAnkle,
            const CenterOfMassComputationPtr_t comc)
        {
          NamedConstraints_t constraints;
          std::vector <NumericalConstraintPtr_t> numericalConstraints =
            createStaticStabilityConstraint (robot, comc, leftAnkle, rightAnkle,
					     *config, false);
          std::string p (prefix);
          constraints.push_back (NamedConstraint_t
              (p + std::string ("/relative-com"),
               numericalConstraints [0]));
          constraints.push_back (NamedConstraint_t
              (p + std::string ("/pose-left-foot"),
               numericalConstraints [1]));
          constraints.push_back (NamedConstraint_t
              (p + std::string ("/pose-right-foot"),
               numericalConstraints [2]));
          return constraints;
        }

        NamedConstraints_t createSliding (const DevicePtr_t robot,
            const char* prefix, const ConfigurationPtr_t& config,
            const JointPtr_t leftAnkle, const JointPtr_t rightAnkle,
            const CenterOfMassComputationPtr_t comc)
        {
          NamedConstraints_t constraints;
          std::vector <NumericalConstraintPtr_t> numericalConstraints =
            createStaticStabilityConstraint (robot, comc, leftAnkle, rightAnkle,
					     *config, true);
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
          numericalConstraints = createSlidingStabilityConstraintComplement
            (robot, leftAnkle, *config);
          const std::string postfix = "-complement";
          constraints.push_back (NamedConstraint_t
              (p + std::string ("/orientation-left-foot") + postfix,
               numericalConstraints [0]));
          constraints.push_back (NamedConstraint_t
              (p + std::string ("/position-left-foot") + postfix,
               numericalConstraints [1]));
          return constraints;
        }

        NamedConstraints_t createAlignedCOM (const DevicePtr_t robot,
            const char* prefix, const ConfigurationPtr_t& config,
            const JointPtr_t leftAnkle, const JointPtr_t rightAnkle,
	    const CenterOfMassComputationPtr_t comc, bool sliding)
        {
          NamedConstraints_t constraints;
          std::vector <NumericalConstraintPtr_t> numericalConstraints =
            createAlignedCOMStabilityConstraint (robot, comc,
						 leftAnkle, rightAnkle, *config,
						 sliding);
          std::string p (prefix);
          constraints.push_back (NamedConstraint_t
				 (p + std::string ("/com-between-feet"),
				  numericalConstraints [0]));
          constraints.push_back (NamedConstraint_t
				 (p + std::string ("/pose-right-foot"),
				  numericalConstraints [1]));
          constraints.push_back (NamedConstraint_t
				 (p + std::string ("/pose-left-foot"),
				  numericalConstraints [2]));
          return constraints;
        }
      }

      static ConfigurationPtr_t dofSeqToConfig
      (ProblemSolverPtr_t problemSolver, const hpp::dofSeq& dofArray)
      {
	CORBA::ULong configDim = dofArray.length();
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
	for (CORBA::ULong iDof=0; iDof < configDim; ++iDof) {
	  (*config) [iDof] = dofArray [iDof];
	}
	return config;
      }

      Problem::Problem () : server_ (0x0) {}

      ProblemSolverPtr_t Problem::problemSolver ()
      {
	return server_->problemSolver ();
      }

      void Problem::addStaticStabilityConstraints
      (const char* prefix, const hpp::dofSeq& dofArray,
       const char* leftAnkle, const char* rightAnkle, const char* comName,
       const StaticStabilityType type)
      throw (hpp::Error)
      {
        using pinocchio::CenterOfMassComputationPtr_t;
	using core::DifferentiableFunctionPtr_t;
	try {
	  ConfigurationPtr_t config = dofSeqToConfig (problemSolver(), dofArray);
	  const DevicePtr_t& robot (problemSolver()->robot ());
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
          } else {
            comc = problemSolver()->centerOfMassComputation (comN);
            if (!comc)
              throw Error ("This CenterOfMassComputation does not exist");
          }
          NamedConstraints_t nc;
          switch ( type ) {
            case hpp::corbaserver::wholebody_step::Problem::SLIDING:
              nc = createSliding (robot, prefix, config, la, ra, comc);
              break;
            case hpp::corbaserver::wholebody_step::Problem::SLIDING_ALIGNED_COM:
              nc = createAlignedCOM (robot, prefix, config, la, ra, comc, true);
              break;
	    case hpp::corbaserver::wholebody_step::Problem::FIXED_ON_THE_GROUND:
	      nc = createFixed (robot, prefix, config, la, ra, comc);
              break;
            case hpp::corbaserver::wholebody_step::Problem::FIXED_ALIGNED_COM:
              nc = createAlignedCOM (robot, prefix, config, la, ra, comc,
				     false);
              break;
            default:	
              throw Error ("Unkown StaticStability type.");
              break;
          }
          for (NamedConstraints_t::const_iterator it = nc.begin ();
              it != nc.end (); ++it) {
            problemSolver()->addNumericalConstraint
              (it->first, it->second);
          }
	} catch (const std::exception& exc) {
	  throw Error (exc.what ());
	}
      }

      void Problem::generateGoalConfig
      (CORBA::Double, CORBA::Double, CORBA::Double,
       CORBA::UShort) throw (hpp::Error)
      {
	assert (problemSolver());
      }
    } // namespace impl
  } // namespace wholebodyStep
} // namespace hpp
