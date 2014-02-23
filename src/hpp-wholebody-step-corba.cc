// Copyright (C) 2013 CNRS-LAAS
// Author: Florent Lamiraux
//
// This file is part of the hpp-wholebody-step-corba.
//
// This software is provided "as is" without warranty of any kind,
// either expressed or implied, including but not limited to the
// implied warranties of fitness for a particular purpose.
//
// See the COPYING file for more information.

#include <iostream>
#include <hpp/util/debug.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/corbaserver/server.hh>
#include <hpp/corbaserver/wholebody-step/server.hh>

typedef hpp::wholebodyStep::Server WholebodyServer;
typedef hpp::corbaServer::Server CorbaServer;
typedef hpp::wholebodyStep::ProblemSolverPtr_t ProblemSolverPtr_t;
typedef hpp::wholebodyStep::ProblemSolver ProblemSolver;
int
main (int argc, char* argv[])
{
  Eigen::internal::set_is_malloc_allowed (true);
  ProblemSolverPtr_t problemSolver = new ProblemSolver;
  CorbaServer corbaServer (problemSolver, argc,
			   const_cast<const char**> (argv), true);
  WholebodyServer wbsServer (argc, argv, true);
  wbsServer.setProblemSolver (problemSolver);

  try {
    corbaServer.startCorbaServer ();
    hppDout (info, "successfully start hpp-corbaserver");
  } catch (const std::exception& exc) {
    hppDout (error, "Faile to start hpp-corbaserver");
  }
  try {
    wbsServer.startCorbaServer ("hpp", "plannerContext",
				"hpp", "wholebodyStep");

    hppDout (info, "Successfully started corba server for whole body planner");
  } catch (const std::exception& exc) {
    hppDout (error, "failed to start corba server for whole body planner");
  }
  corbaServer.processRequest(true);
}
