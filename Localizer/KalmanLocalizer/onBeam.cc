#include "kalmanlocalizer.ih"

void KalmanLocalizer::onBeam(shared_ptr<BeamEvent> event)
{
  d_globalRotation = AngleAxisd(event->getWhere()[2] / 180.0 * M_PI -.5 * M_PI, Vector3d::UnitZ());

  Vector3d pos = event->getWhere();

  //
  // When we beam, our Z position is fixed.
  //
  pos.z() = 0.35;
  
//   if ((pos - me->getPositionGlobal()).norm() > 0.5)
//   {
    MatrixXd s2 = MatrixXd::Identity(6, 6) * 1;
    VectorXd m = VectorXd::Zero(6);
    d_me->posVelGlobal->init(joinPositionAndVelocityVectors(pos, Vector3d(0,0,0)), s2);
    d_ball->posVelGlobal->init(m, s2);
//     for (shared_ptr<PlayerInfo> player : players)
//     {
//       m = player->posVelGlobal->getMu();
//       player->posVelGlobal->init(m, s2);
//     }
//   }
}

