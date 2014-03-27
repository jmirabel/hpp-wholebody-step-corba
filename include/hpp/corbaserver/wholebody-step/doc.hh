/** \mainpage 

\section hppWholebodyStepServer_sec_intro Introduction

This package implements a Corba interface with hpp-corbaserver package. As for the hppcorbaserver executable,  requests can be sent to trigger actions in a hpp::core::ProblemSolver object. 

Four main Corba interfaces are implemented. For the three Corba inferfaces hpp::Robot:, hpp::Obstacle: and hpp::Problem:, please refer to hpp::corbaServer:. The last one is :
\li hpp::WholebodyStep: to add static stability constraints and generate a goal configuration.


\section hppWholebodyStepServer_sec_howto How to communicate with the CORBA server

The easiest way is
\li to launch hpp-wholebody-step-server executable or any executable that implements the server,
\li open a python terminal and type:
\code
from hpp.corbaserver.wholebody_step.client import Client as WsClient
wcl = WsClient ()
\endcode

Then variable \c wcl contains the problem member that can send constraint request to the server, usign the method hpp::WholebodyStep::addStaticStabilityConstraints(in dofSeq dofArray), for example as it follows :
\code
wcl.problem.addStaticStabilityConstraints (q0)	
\endcode

hpp::WholebodyStep::generateGoalConfig method is not yet implemented.

\section hppWholebodyStepServer_sec_embedding How to embed a server in an application

To embed a CORBA server in an application, See documentation of class hpp::wholebodyStep::Server.

*/
