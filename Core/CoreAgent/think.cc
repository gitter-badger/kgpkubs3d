#include "Coreagent.ih"
#include "IKCCD.hh"
//#include "Walk_OC.hh"

void CoreAgent::think()
{
  WorldModel& wm = SWorldModel::getInstance();
  Cochlea& cochlea = SCochlea::getInstance();
  AgentSocketComm& comm = SAgentSocketComm::getInstance();
  Localizer& loc = SLocalizer::getInstance();
  AgentModel& am = SAgentModel::getInstance();
  Cerebellum& cer = SCerebellum::getInstance();
  double t = wm.getTime();

  //walk_oc woc;

  InverseKinematics lhand(am,cochlea,comm,cer,wm,loc,Types::LEFTHAND);
  lhand.run(Vector3d(-0.25,0.14,0.084));

  InverseKinematics rhand(am,cochlea,comm,cer,wm,loc,Types::RIGHTHAND);
  rhand.run(Vector3d(0.25,0.14,0.084));

  //map <Types::Joint, double >  target = woc.genAngles();

  //for(auto it:target)
//    cer.addAction( make_shared<MoveJointAction>( it.first,it.second) );

//  cer.outputCommands(comm);
}