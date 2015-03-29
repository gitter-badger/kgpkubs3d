#include "CoreAgent/Coreagent.hh"
int main()
{
  util::Thread *agent = new CoreAgent();
 // util::Thread *agent1 = new CoreAgent();
 //Thread *t = new Thread(agent[0]);
  agent->start();
 // agent1->start();
  /*Thread *t1 = new Thread(agent[1]);
  t1->run();
  Thread *t2 = new Thread(agent[2]);
  t2->run();
  Thread *t3 = new Thread(agent[3]);
  t3->run();
  Thread *t4 = new Thread(agent[4]);
  t4->run();*/
}

