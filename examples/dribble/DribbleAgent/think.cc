#include "dribbleagent.ih"

void DribbleAgent::think()
{
  AgentModel& am = SAgentModel::getInstance();
  WorldModel& wm = SWorldModel::getInstance();
  Cerebellum& cer = SCerebellum::getInstance();
  		  Vector3d des=SLocalizer::getInstance().getOurGoalMidpointGlobal();
 Localizer& localizer = SLocalizer::getInstance();
  VectorXd jointVelocities;
/*
  if (!d_gettingUpFrom)
  {
    if (am.onMyBack())
      d_gettingUpFrom = 1;
    else if (am.onMyBelly())
      d_gettingUpFrom = 2;
  }

  if (d_gettingUpFrom)
  {
    /**********
     * GETTING UP
     **********/
  /*  auto motionSequencePlayer = d_motionSequencePlayers[d_gettingUpFrom == 1 ? "getupfromback" : "getupfromfront"];
    motionSequencePlayer->run(0);
    jointVelocities = motionSequencePlayer->getJointVelocities();

    if (motionSequencePlayer->isSequenceDone())
    {
      motionSequencePlayer->reset();
      d_gettingUpFrom = 0;
    }
  }
  else
  {
    if (wm.getPlayMode() != Types::PLAY_ON)
    {
      /**********
       * STANDING
       **********/
/*      jointVelocities = stand();

      if (wm.getPlayMode() == Types::BEFORE_KICKOFF && !d_beamed)
      {
	cer.addAction(make_shared<BeamAction>(Vector3d(-3, 0, 0)));
	d_beamed = true;
      }

    }
    else
    {
      /**********
       * WALKING
       **********/
  /*     #ifdef orig
      VectorXd whereToWalkTo = determineWhereToWalk();

      // Initialize gait generator parameters
      GaitParams gaitParameters;
      gaitParameters.params = whereToWalkTo;

      // Run gait generator
      d_gaitGenerator->run(&gaitParameters);

      // Get results
      jointVelocities = d_gaitGenerator->getJointVelocities();
      #endif*/
      static bool reached=false;
      if(!reached)
      {
		  des(0)+= 1;  //to prevent collision from goalpost
		  //jointVelocities = goToPoint(localizer.globalToLocal(des));
			goToPoint(localizer.globalToLocal(des));

		  if(localizer.globalToLocal(des).norm()<0.5)
		  { reached = true; cout <<"reached goal mid\n"; }
	  }

	  else /*startif now we have reached centre of our goal*/
	  {

		  bool do_once = false;
		  if(!do_once)
		  {
			  des(1)+=1; do_once= true;
		  }
		  static Vector3d ball = localizer.getBall()->getPositionGlobal(true);
		  cout <<"("<<ball(0)<<","<<ball(1)<<","<<ball(2)<<")\n";
		  //Vector3d ball(6.7,15,0);
		  if(ball(1) >-14)
		  {
			ball(1)+= 0.005;
		  }

		  des(1) = ball(1);
		  cout <<"here1\n";
		  static Vector3d ourGoal =  localizer.getOurGoalMidpointGlobal();
		  static double goalBoundary1 = ourGoal(1) + wm.getGoalWidth()/2;
		  static double goalBoundary2 = ourGoal(1) - wm.getGoalWidth()/2;

		  cout <<"here2\n";
		  cout << goalBoundary1<<" " << goalBoundary2 << " "<<des(1)<<"\n";
		  if((des(1) < goalBoundary1) && (des(1) >goalBoundary2) )
		  {
			  cout <<"here3\n";
				  //jointVelocities = goToPoint(localizer.globalToLocal(des));
				  goToPoint(localizer.globalToLocal(des));
		  }
		  else if(des(1) > goalBoundary1)
		  {
			  Vector3d des2(des(0),goalBoundary1,0);
			  //jointVelocities = goToPoint(localizer.globalToLocal(des2));
			  goToPoint(localizer.globalToLocal(des2));
		  }
		  else if(des(1) < goalBoundary2)
		  {
			  Vector3d des2(des(0),goalBoundary2,0);
			 // jointVelocities = goToPoint(localizer.globalToLocal(des2));
			 goToPoint(localizer.globalToLocal(des2));
		  }
		  else
		        jointVelocities = stand();

		  cout << "here4\n";
//		  jointVelocities = goToPoint(localizer.globalToLocal(des));
	  }  /*endif now we have reached centre of our goal*/


    } ///close all ur stuff before here

    /**********
     * LOOKING
     **********/
    /*VectorXd currentJointAngles = am.getJointAngles();

    Vector2d whereToLookAt = determineWhereToLook();
    jointVelocities(Types::HEAD1) = whereToLookAt(0) - currentJointAngles(Types::HEAD1);
    jointVelocities(Types::HEAD2) = whereToLookAt(1) - currentJointAngles(Types::HEAD2);
  }

  // Add actions to the Cerebellum
  for (unsigned j = 0; j < Types::NJOINTS; ++j)
    cer.addAction(make_shared<MoveJointAction>((Types::Joint)j, jointVelocities(j)));

  // Tell Cerebellum to send the actions to the server
  cer.outputCommands(SAgentSocketComm::getInstance());

  //
  // Notify AgentModel of control
  //
  am.setControl(jointVelocities);
}*/

