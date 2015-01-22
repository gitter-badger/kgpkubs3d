#include "worldmodel.ih"

unsigned WorldModel::otherTeamMateCloserTo(Vector3d const& posLocal, bool standingOnly, bool returnCount)
{
  Localizer& loc = bats::SLocalizer::getInstance();

  double dist = posLocal.norm();
  unsigned count = 0;

  for (shared_ptr<PlayerInfo> teamMate : loc.getTeamMates())
  {
    if (!teamMate->isAlive || teamMate->isMe)
      continue;
    
    Vector3d teamMatePosLocal = teamMate->getPositionLocal(/*ZeroZ*/true);

    double d = (teamMatePosLocal - posLocal).norm();

    if (d >= dist)
      continue;
    
    if (standingOnly && teamMate->fallen)
      continue;
    
    if (!returnCount)
      return true;

    ++count;
  }

  return count;
}

unsigned WorldModel::otherTeamMateCloserToBall(bool standingOnly, bool returnCount)
{
  Localizer& loc = bats::SLocalizer::getInstance();

  return otherTeamMateCloserTo(loc.getBall()->getPositionLocal(/*zeroZ*/true), standingOnly, returnCount);
}
