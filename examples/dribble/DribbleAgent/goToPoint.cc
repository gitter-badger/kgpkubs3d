//TO_DO: figure out what params.tail does 
#include "dribbleagent.ih"
// the pointPosition is itself a Localized position
VectorXd DribbleAgent::goToPoint(const Vector3d& pointPosition)
{
  VectorXd params(6);

  Vector3d selfPosition(0,0,0);

  params.head<3>() = pointPosition.normalized() ;			// Point co-ordinates where we have to go                  
  params.tail<3>() = pointPosition.normalized() * 2;			//still to figure out.

  d_paramFilter += 0.05 * (params - d_paramFilter);			// to avoid abrupt changes we normalize it here
  // initializing GaitParams
  GaitParams gaitParameters;
  gaitParameters.params = d_paramFilter;
  // Run gait generator
  d_gaitGenerator->run(&gaitParameters);
  // Get results
  VectorXd jointVelocities = d_gaitGenerator->getJointVelocities();
  return jointVelocities;
 
}
