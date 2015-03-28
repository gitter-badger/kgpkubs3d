//TO_DO: figure out what params.tail does 
#include "Coreagent.ih"
/* side = 0 => right side  *
 * side = 1 => front side  *
 * side = 2 => left side   *
 * side = 3 => back side   */
void CoreAgent::fallNow(int sideToFall) 
{
  Vector3d pointPosition;
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
    return; //fallen
  }

  else
  {

      VectorXd params(6);

      if(sideToFall == 0)
          pointPosition = Vector3d(2,0,0);
      else if(sideToFall == 1)
          pointPosition = Vector3d(0,2,0);
      else if(sideToFall == 2)
          pointPosition = Vector3d(-2,0,0);
      else if(sideToFall == 3)
          pointPosition = Vector3d(0,-2,0);
      else return;

      params.head<3>() = pointPosition.normalized() ;			// Point co-ordinates where we have to go                  
      params.tail<3>() = pointPosition.normalized() ;			//still to figure out.
      d_paramFilter += 0.0001 * (params - d_paramFilter);  
      GaitParams gaitParameters;
      gaitParameters.params = d_paramFilter;
      d_gaitGenerator->run(&gaitParameters);
      jointVelocities = d_gaitGenerator->getJointVelocities();
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
