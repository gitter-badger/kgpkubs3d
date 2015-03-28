//TO_DO: figure out what params.tail does 
#include "Coreagent.ih"
// the pointPosition is itself a Localized position
void CoreAgent::goToPoint(const Vector3d& pointPosition)
{
  AgentModel& am = SAgentModel::getInstance();
  Cerebellum& cer = SCerebellum::getInstance();
  VectorXd jointVelocities; 
  WorldModel& wm = SWorldModel::getInstance();


//////If bot is fallen
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
    auto motionSequencePlayer = d_motionSequencePlayers[d_gettingUpFrom == 1 ? "getupfromback" : "getupfromfront"];
    motionSequencePlayer->run(0);
    jointVelocities = motionSequencePlayer->getJointVelocities();

    if (motionSequencePlayer->isSequenceDone())
    {
      motionSequencePlayer->reset();
      d_gettingUpFrom = 0;
    }
  }
//////////bot fallen or not code ends 
else
{
  if(!(pointPosition.norm()<0.5))
  {
	/////////WALKING
  VectorXd params(6);

  Vector3d selfPosition(0,0,0);

  params.head<3>() = pointPosition.normalized() ;			// Point co-ordinates where we have to go                  
  params.tail<3>() = pointPosition.normalized() ;			//still to figure out.

  d_paramFilter += 0.05 * (params - d_paramFilter);			// to avoid abrupt changes we normalize it here
  // initializing GaitParams
  GaitParams gaitParameters;
  gaitParameters.params = d_paramFilter;
  // Run gait generator
  d_gaitGenerator->run(&gaitParameters);
  // Get results
  jointVelocities = d_gaitGenerator->getJointVelocities();
  }

  else
  {
	////////STANDING
	jointVelocities = stand();
	if (wm.getPlayMode() == Types::BEFORE_KICKOFF && !d_beamed)    //what's its meaning ???
      {
	cer.addAction(make_shared<BeamAction>(Vector3d(-3, 0, 0)));
	d_beamed = true;
      }
  }
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
 
}
