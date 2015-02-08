#include "helloworldagent.ih"

const float rtd = 180/M_PI;
const float dtr = M_PI/180;

void print(string name,float calc,float actual)
{
  cout << name << " " << rtd*calc << " " << rtd*actual << endl;
}

void inverse_kinematic(AgentModel& am,Cerebellum& cer)
{
  Eigen::Affine3d td = (am.getJoint(Types::LLEG1)->transform).inverse(); //
  Eigen::Vector3d pd = td * am.getJointPosition(Types::LLEG6); //

  double l1 = 0.11005; //
  double l2 = 0.11005; //
  double d = pd.norm();
  cout << l1 << " " << l2 << " " << d << endl;

  double th4 = - M_PI + acos((l1*l1 + l2*l2 - d*d)/(2*l1*l2)); 
  double pdx = pd[0];
  double pdy = pd[1];
  double pdz = pd[2];
  double th6 = atan2(abs(pdy),abs(pdz));
  double num = pdy*(l2 + l1*cos(th4)) + l1*pdx*sin(th4);
  double denum = l1*l1*sin(th4)*sin(th4)+(l2+l1*cos(th4))*(l2+l1*cos(th4));
  double th5 = asin(-num/denum);

  cout << pdx << " " << pdy << " " << pdz << endl;

  Eigen::Affine3d tddd = (am.getJoint(Types::LLEG3)->transform); //
  double th2 = acos(tddd(2,3)) - M_PI/4;
  cout << tddd(2,2) << " " << sin(th2 + M_PI/2) << endl;
  double th3 = asin( tddd(2,2) / sin(th2 + M_PI/4) );
  double th1 = acos(tddd(1,3)/sin(th2+M_PI/4)) + M_PI/2;

  print("1",th1,am.getJoint(Types::LLEG1)->angle->getMu()(0));
  print("2",th2,am.getJoint(Types::LLEG2)->angle->getMu()(0));
  print("3",th3,am.getJoint(Types::LLEG3)->angle->getMu()(0));
  print("4",th4,am.getJoint(Types::LLEG4)->angle->getMu()(0));
  print("5",th5,am.getJoint(Types::LLEG5)->angle->getMu()(0));
  print("6",th6,am.getJoint(Types::LLEG6)->angle->getMu()(0));
}

double l=0;
const double interval = .02;

// put inside a while loop
Eigen::Vector3d trajectory()
{
   l = (l+interval)>1?(l+interval)-1:(l+interval);  // using mod on decimal was giving error..
   int ld = 1-l;

  Eigen::Vector3d al,ar,sllift,srlift,sl,sr; // input.. we might need to decide suitable values

  double tllift,trlift,tlmove,trmove,tl,tr;
  int x1=0;
  int y1=.5;
  int xm=0;
  int ym=.5;
  if(2*l>=x1 && 2*l<=(x1+y1))
    tllift = .5*(1-cos(2*M_PI*((2*l-x1)/y1)));
  else
    tllift = 0;
  if(2*ld>=x1 && 2*ld<=(x1+y1))
    trlift = .5*(1-cos(2*M_PI*((2*ld-x1)/y1)));
  else
    trlift = 0;
  if(2*l>=x1 && 2*l<=(x1+y1))
    tlmove = .5*(1-cos(M_PI*((2*l-x1)/y1)));
  else if(2*l>(x1+y1) && 2*l<=1)
    tlmove = 1;
  else
    tlmove =0;
  if(2*ld>=x1 && 2*ld<=(x1+y1))
    trmove = .5*(1-cos(M_PI*((2*l-x1)/y1)));
  else if(2*ld>(x1+y1) && 2*ld<=1)
    trmove = 1;
  else
    trmove =0;
  if(l<0.5)
    tl = .5*(1-cos(2*M_PI*l));
  else
    tl=0;
  if(l>=0.5)
    tr = .5*(1-cos(2*M_PI*l));
  else
    tr=0;

     // foot positions relative to the torso

  Eigen::Vector3d plrel,prrel;    

  if(l<0.5)
    plrel = al + sl*tlmove + sllift*tllift - sr*(1-tlmove)*tl;
  else
    plrel = al + sllift*tllift + sl*(1-tr);
  if(l>=0.5)
    prrel = ar + sr*trmove + srlift*trlift - sl*(1-trmove)*tr;
  else
    prrel = ar + srlift*trlift + sr*(1-tl);

  // for foot position w.r.t. COM

  Eigen::Vector3d cs ;                     // offset of COM relative to torso - from sim..
  Eigen::Vector3d ss ;                     // amplitude of COM in y direction - input
  double xc,yc,zc;
  double sp = sin(2*M_PI*l);
  double rp = sqrt(2*M_PI*l);
  double lp = l<.25?4*l : (l>=.75? 4*(l-1) : (2-4*l));
  double tcom,tlin;
  tcom = (xc*sp + yc*rp + zc*lp)/( xc+yc+zc);
  tlin = l<0.5 ? 2*l : 2*l-1;

  Eigen::Vector3d plcom,prcom;

  if(l<0.5)
    plcom = (cs*-1 + ss*tcom + al - (sl*tlin -  sr*(1-tlin)))*0.5;
  else
    plcom = prcom - prrel + plrel;

  if(l<0.5)
    prcom = (cs*-1 + ss*tcom + ar - (sr*tlin -  sl*(1-tlin)))*0.5;
  else
    prcom = plcom - plrel + prrel;

  Eigen::Vector3d enew = (plrel - plcom + prrel - prcom)*0.5 ;        // one parameter missing(the offset of COM w.t.r. torso , need to confirm 1 thing here! )

  Eigen::Vector3d final_l = plrel - enew;
  Eigen::Vector3d final_r = prrel - enew;
  // we will use these final values and apply IK on them..
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

  //inverse_kinematic(am,cer);
/*
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
  tla[4] = -knee;//knee*sin(theta)-knee - constant;
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
    vla[i] = 0.01*(tla[i]-la[i]);
    vra[i] = 0.1*(tra[i]-ra[i]);
  }

  print("leg4",tla[4],la[4]);
  cout << vla[4] << endl;

  // cer.addAction(make_shared<MoveJointAction>(Types::LLEG2, vla[2]));
  // cer.addAction(make_shared<MoveJointAction>(Types::LLEG3, vla[3]));
  cer.addAction(make_shared<MoveJointAction>(Types::LLEG4, vla[4]));
  // cer.addAction(make_shared<MoveJointAction>(Types::LLEG5, vla[5]));
  // cer.addAction(make_shared<MoveJointAction>(Types::LLEG6, vla[6]));

  // cer.addAction(make_shared<MoveJointAction>(Types::RLEG2, vra[2]));
  // cer.addAction(make_shared<MoveJointAction>(Types::RLEG3, vra[3]));
  // cer.addAction(make_shared<MoveJointAction>(Types::RLEG4, vra[4]));
  // cer.addAction(make_shared<MoveJointAction>(Types::RLEG5, vra[5]));
  // cer.addAction(make_shared<MoveJointAction>(Types::RLEG6, vra[6]));

  // Tell Cerebellum to send the actions to the server
  cer.outputCommands(SAgentSocketComm::getInstance());
  */
}