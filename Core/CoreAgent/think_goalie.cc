#include "Coreagent.ih"

void CoreAgent::think()
{
  static const double constGoalieMin = MacroGoalieLimit - 1.0; 
  AgentModel& am = SAgentModel::getInstance();
  WorldModel& wm = SWorldModel::getInstance();
  Cerebellum& cer = SCerebellum::getInstance();
  Vector3d des=SLocalizer::getInstance().getOurGoalMidpointGlobal();
  Localizer& localizer = SLocalizer::getInstance();
  VectorXd jointVelocities;
  static bool reached = false;
  if(!reached)
  {
      des(0)+= 1.5;  //to prevent collision from goalpost
      //jointVelocities = goToPoint(localizer.globalToLocal(des));
      goToPoint(localizer.globalToLocal(des));

      if(localizer.globalToLocal(des).norm()<0.5)
      { 
         reached = true; cout <<"reached goal mid\n"; 
      }

  }

  else /*startif now we have reached centre of our goal*/
  {
      Vector3d ball = localizer.getBall()->getPositionGlobal(true);

      des(1) = ball(1);

      static Vector3d ourGoal =  localizer.getOurGoalMidpointGlobal();
      static double goalBoundary1 = ourGoal(1) + wm.getGoalWidth()/2;
      static double goalBoundary2 = ourGoal(1) - wm.getGoalWidth()/2;

      /*WHILE ball is till -12 Goalie take ball to opp's goal *
       *From -12 to -7 defender will take care of ball        *
       *After -7 mid fielder will take the ball               */
      if(ball(0) > MacroGoalieLimit) ///IF Ball is far away go back to goal post
      {
          Vector3d des2(-13.0,ourGoal(1),0.0);
          goToPoint(localizer.globalToLocal(des2));
      }
      else if((des(1) < goalBoundary1) && (des(1) >goalBoundary2) && ball(0) < -12) 
      {
         takeBallToGoal();
      }
      else if(des(1) > goalBoundary1)
      {
        double des2_0 =des(0);
        if(ball(0)>0) //ball is in opponent's half
            des2_0 = constGoalieMin;
        Vector3d des2(des2_0,goalBoundary1,0);
        goToPoint(localizer.globalToLocal(des2));
      }
      else if(des(1) < goalBoundary2)
      {
        Vector3d des2(des(0),goalBoundary2,0);
        goToPoint(localizer.globalToLocal(des2));
      }
      else ///their must be no else actually !
      {
        Vector3d des2(constGoalieMin,ourGoal(1),0.0);
        goToPoint(localizer.globalToLocal(des2));
      }
  }  


}
