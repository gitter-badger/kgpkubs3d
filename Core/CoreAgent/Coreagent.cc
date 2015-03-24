#include "Coreagent.ih"

CoreAgent::CoreAgent()
  : HumanoidAgent("Core", "../xml/conf.xml"),
    d_beamed(false)
{
}
