#include "Coreagent.ih"

CoreAgent::CoreAgent()
  : HumanoidAgent("Core", "./conf.xml"),
    d_beamed(false)
{
}
