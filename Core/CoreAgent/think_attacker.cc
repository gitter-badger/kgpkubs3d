#include "Coreagent.ih"


void CoreAgent::think_attacker()
{
	static const double attackerNormalPos0 = 0;
	WorldModel& wm = SWorldModel::getInstance();
	Vector3d des(attackerNormalPos0,0,0);
	Localizer& localizer = SLocalizer::getInstance();

	static bool reached=false;
	if(!reached)
	{
		goToPoint(localizer.globalToLocal(des));
		if(localizer.globalToLocal(des).norm()<0.5)
		{ 
			reached = true; cout <<"reached attackerNormalPos0 mid\n"; 
		}
	}
      /*WHILE ball is till -12 Goalie take ball to opp's goal  *
       *From -12 to -7 defender will take care of ball         *
       *After -7 mid fielder will take the ball                */
	else 
	{
		Vector3d ball = localizer.getBall()->getPositionGlobal(true);

		if(ball(0)>MacroattackerLimit || ball(0) < (MacroattackerLimit + 0.5)) //Ball is far, reset position !
		{
			static Vector3d ourGoal =  localizer.getOurGoalMidpointGlobal();
			int x = ball(1) - ((ball(1)-ourGoal(1))/(ball(0)-ourGoal(0))) * ball(0);
/*			cout <<"x = " << x << "\n";
			cout <<ball(0) <<" "<< ball(1) << " " << ball(2) <<"\n"; 
 */			goToPoint(localizer.globalToLocal(Vector3d(0,x,0)));
		}
		else
		{
//			cout << "ELSE: ball(0)= "<<ball(0)<<"\n";
			takeBallToGoal();
		}
	}
} 
