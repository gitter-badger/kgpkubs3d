#include "helloworldagent.ih"

void print(string name,float calc,float actual)
{
  cout << name << " " << (180*calc)/M_PI << " " << (180*actual)/M_PI << endl;
}

void inverse_kinematic(AgentModel& am,Cerebellum& cer)
{
  Eigen::Affine3d td = (am.getJoint(Types::LLEG1)->transform).inverse(); //
  Eigen::Vector3d pd = td * am.getJointPosition(Types::LLEG6); //

  double l1 = 0.110001; //
  double l2 = 0.110001; //
  double d = pd.norm();
  cout << l1 << " " << l2 << " " << d << endl;

  double th4 = M_PI - acos((l1*l1 + l2*l2 - d*d)/(2*l1*l2)); 
  double pdx = pd[0];
  double pdy = pd[1];
  double pdz = pd[2];
  double th6 = atan2(pdy,pdz);
  double num = pdy*(l2 + l1*cos(th4)) + l1*pdx*sin(th4);
  double denum = l1*l1*sin(th4)*sin(th4)+(l2+l1*cos(th4))*(l2+l1*cos(th4));
  double th5 = asin(num/denum);

  cout << pdx << " " << pdy << " " << pdz << endl;

  Eigen::Affine3d tddd = (am.getJoint(Types::LLEG3)->transform); //
  double th2_cap = acos(tddd(2,3));
  double th2 = th2_cap - M_PI/4;
  double th3 = asin( tddd(2,2) / sin(th2 + M_PI/2) );
  double th1_cap = acos(tddd(1,3)/sin(th2+M_PI/4));
  double th1 = th1_cap + M_PI/2;

  print("1",th1,am.getJoint(Types::LLEG1)->angle->getMu()(0));
  print("2",th2,am.getJoint(Types::LLEG2)->angle->getMu()(0));
  print("3",th3,am.getJoint(Types::LLEG3)->angle->getMu()(0));
  print("4",th4,am.getJoint(Types::LLEG4)->angle->getMu()(0));
  print("5",th5,am.getJoint(Types::LLEG5)->angle->getMu()(0));
  print("6",th6,am.getJoint(Types::LLEG6)->angle->getMu()(0));
}

void HelloWorldAgent::think()
{
  // The WorldModel keeps track of the state of the world, e.g. play mode, time,
  // players/opponents/ankle_tiltl positions, et cetera
  WorldModel& wm = SWorldModel::getInstance();
  
  // The AgentModel keeps track of the state of the robot, e.g. joint angles,
  // position of COM, et cetera
  AgentModel& am = SAgentModel::getInstance();
  
  // The Cerebellum collects actions to perform, integrating them where necesary
  Cerebellum& cer = SCerebellum::getInstance();
  
  // Get the current time
  double t = wm.getTime();

  inverse_kinematic(am,cer);

  // Get the current angles of some shoulder joints
  double la[7],ra[7];
  la[1] = am.getJoint(Types::LLEG1)->angle->getMu()(0);
  la[2] = am.getJoint(Types::LLEG2)->angle->getMu()(0);
  la[3] = am.getJoint(Types::LLEG3)->angle->getMu()(0);
  la[4] = am.getJoint(Types::LLEG4)->angle->getMu()(0);
  la[5] = am.getJoint(Types::LLEG5)->angle->getMu()(0);
  la[6] = am.getJoint(Types::LLEG6)->angle->getMu()(0);
  ra[1] = am.getJoint(Types::RLEG1)->angle->getMu()(0);
  ra[2] = am.getJoint(Types::RLEG2)->angle->getMu()(0);
  ra[3] = am.getJoint(Types::RLEG3)->angle->getMu()(0);
  ra[4] = am.getJoint(Types::RLEG4)->angle->getMu()(0);
  ra[5] = am.getJoint(Types::RLEG5)->angle->getMu()(0);
  ra[6] = am.getJoint(Types::RLEG6)->angle->getMu()(0);

  double tla[7],tra[7];
  double knee=M_PI/4 ,hip=M_PI/4 ,ankle=M_PI/12 ,ankle_tilt=M_PI/9 ,hip_tilt = M_PI/9;
  //cout << t << endl;
  double omega = 7*M_PI;
  double theta = t*omega;
  double stheta = theta + M_PI/2;
  double constant = 0;
  //cout << t << " " << theta << endl;
  double otheta = theta + M_PI;
  double ostheta = otheta + M_PI/2;
  // left body
  tla[2] = hip*sin(otheta)+hip;
  tla[3] = hip_tilt*sin(otheta);
  tla[4] = knee*sin(theta)-knee - constant;
  tla[5] = ankle*sin(otheta)+ankle;
  tla[6] = ankle_tilt*sin(otheta);
  // right body
  tra[2] = hip*sin(theta)+hip;
  tra[3] = hip_tilt*sin(otheta);
  tra[4] = knee*sin(otheta)-knee - constant;
  tra[5] = ankle*sin(theta)+ankle;
  tra[6] = ankle_tilt*sin(otheta);

  double vla[7],vra[7];
  for(int i=0;i<7;i++)
  {
    vla[i] = 0.1*(tla[i]-la[i]);
    vra[i] = 0.1*(tra[i]-ra[i]);
  }

  // cer.addAction(make_shared<MoveJointAction>(Types::LLEG2, vla[2]));
  // cer.addAction(make_shared<MoveJointAction>(Types::LLEG3, vla[3]));
  //cer.addAction(make_shared<MoveJointAction>(Types::LLEG4, vla[4]));
  // cer.addAction(make_shared<MoveJointAction>(Types::LLEG5, vla[5]));
  // cer.addAction(make_shared<MoveJointAction>(Types::LLEG6, vla[6]));

  // cer.addAction(make_shared<MoveJointAction>(Types::RLEG2, vra[2]));
  // cer.addAction(make_shared<MoveJointAction>(Types::RLEG3, vra[3]));
  // cer.addAction(make_shared<MoveJointAction>(Types::RLEG4, vra[4]));
  // cer.addAction(make_shared<MoveJointAction>(Types::RLEG5, vra[5]));
  // cer.addAction(make_shared<MoveJointAction>(Types::RLEG6, vra[6]));

  // Tell Cerebellum to send the actions to the server
  cer.outputCommands(SAgentSocketComm::getInstance());
}