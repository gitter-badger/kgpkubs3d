//TO_DO: figure out what params.tail does 
#include "Coreagent.ih"
// the pointPosition is itself a Localized position
void CoreAgent::goToPoint_FaceTheGoal(const Vector3d& pointPosition)
{
  AgentModel& am = SAgentModel::getInstance();
  Cerebellum& cer = SCerebellum::getInstance();
  Localizer& localizer = SLocalizer::getInstance();
	
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
   Vector3d selfPosition(0,0,0);
  Vector3d origin_global(0,0,0);
  bool onRightSide = Math::atSameSideOf(selfPosition, pointPosition, localizer.globalToLocal(origin_global));
  cout<<"\n on right side bool "<<onRightSide;
  if(pointPosition.norm()<0.5 && onRightSide)
  {
	////////STANDING
	jointVelocities = stand();
	if (wm.getPlayMode() == Types::BEFORE_KICKOFF && !d_beamed)//////////////////////what's its meaning
      {
	cer.addAction(make_shared<BeamAction>(Vector3d(-3, 0, 0)));
	d_beamed = true;
      }
  }
  else
  {
	/////////WALKING
  VectorXd params(6);

 
  if(onRightSide)
  {
  params.head<3>() = pointPosition.normalized() ;			// Point co-ordinates where we have to go                  
  params.tail<3>() = pointPosition.normalized() ;			//still to figure out.
  }
  else
  {
  Vector3d desToCenterDirection = (localizer.globalToLocal(origin_global) - pointPosition).normalized();
    Vector3d behindDesPosition = pointPosition - 0.25 * desToCenterDirection;
    params.head<3>() = behindDesPosition;
    
    // Face somewhere between the position behind the ball, and the
    // target, depending on how close we are to the ball.
    double faceFactor = behindDesPosition.norm() / 0.5;
    faceFactor = Math::saturate(faceFactor, 0.0, 1.0);

    params.tail<3>() =faceFactor * behindDesPosition.normalized() +
      (1.0 - faceFactor) * localizer.globalToLocal(origin_global).normalized();
  }
  d_paramFilter += 0.05 * (params - d_paramFilter);			// to avoid abrupt changes we normalize it here
  // initializing GaitParams
  GaitParams gaitParameters;
  gaitParameters.params = d_paramFilter;
  // Run gait generator
  d_gaitGenerator->run(&gaitParameters);
  // Get results
  jointVelocities = d_gaitGenerator->getJointVelocities();
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
