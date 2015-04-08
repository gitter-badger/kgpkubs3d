//TO_DO: figure out what params.tail does 
#include "Coreagent.ih"
// the pointPosition is itself a Localized position
void CoreAgent::think()
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
    cout <<"Enter side to fall";
    int sidesss;cin >> sidesss;
    fallNow(sidesss);
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
