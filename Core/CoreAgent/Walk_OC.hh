#ifndef __WALKOC_HH_
#define __WALKOC_HH_

#include <vector>
#include <map>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <queue>
#include <stack>
#include <cmath>
#include <list>
#include <ctime>
#include <cmath>
#include "../../Distribution/NormalDistribution/normaldistribution.hh"
#include "../../Singleton/singleton.hh"
#include "../../Types/types.hh"
#include "../../Math/math.hh"
#include "../../AgentModel/agentmodel.hh"

using namespace std;
using namespace Eigen;

namespace bats
{
    class walk_oc
    {
    private:
        double A[7];
        double C[7];
        double P[7];
        float T = M_PI;
        map <Types::Joint, double > target;
        void init();
        clock_t start;
        clock_t end;
    public:
        double diffclock(clock_t clock1,clock_t clock2)
        {
            double diffticks=clock1-clock2;
            double diffs=(diffticks)/CLOCKS_PER_SEC;
            return diffs;
        }
        walk_oc()
        {
            init();
        }
        map <Types::Joint, double > genAngles();
    };

    void walk_oc::init()
    {
        target[Types::HEAD1] = 0;
        target[Types::HEAD2] = 0;
        target[Types::LARM1] = 0;
        target[Types::LARM2] = 0;
        target[Types::LARM3] = 0;
        target[Types::LARM4] = 0;
        target[Types::RARM1] = 0;
        target[Types::RARM2] = 0;
        target[Types::RARM3] = 0;
        target[Types::RARM4] = 0;
        target[Types::LLEG1] = 0;
        target[Types::LLEG2] = 0;
        target[Types::LLEG3] = 0;
        target[Types::LLEG4] = 0;
        target[Types::LLEG5] = 0;
        target[Types::LLEG6] = 0;
        target[Types::RLEG1] = 0;
        target[Types::RLEG2] = 0;
        target[Types::RLEG3] = 0;
        target[Types::RLEG4] = 0;
        target[Types::RLEG5] = 0;
        target[Types::RLEG6] = 0;

        A[1] = 57.1842, P[1] = 2.9594 ,C[1] = -88.4624;
        A[2] = 5.6445, P[2] = -2.2855 ,C[2] = 3.6390;
        A[3] = 57.1211, P[3] = 0.0887 ,C[3] = 35.9536;
        A[4] = 39.6205, P[4] = -1.8292 ,C[4] = -39.9481;
        A[5] = 46.6315, P[5] = 1.7640 ,C[5] = 28.5095;
        A[6] = 3.7947, P[6] = -1.2067 ,C[6] = -2.9360;

        static clock_t temp = clock();
        start = temp;
    }

    map <Types::Types::Joint,double > walk_oc::genAngles()
    {
        double t = diffclock(clock(),start);
        target[Types::LARM1]= C[1] + A[1]*sin((2.0*M_PI*t)/T + P[1]);
        target[Types::LLEG2]= C[2] + A[2]*sin((2.0*M_PI*t)/T + P[2]);
        target[Types::LLEG3]= C[3] + A[3]*sin((2.0*M_PI*t)/T + P[3]);
        target[Types::LLEG4]= C[4] + A[4]*sin((2.0*M_PI*t)/T + P[4]);
        target[Types::LLEG5]= C[5] + A[5]*sin((2.0*M_PI*t)/T + P[5]);
        target[Types::LLEG6]= C[6] + A[6]*sin((2.0*M_PI*t)/T + P[6]);

        target[Types::RARM1]= C[1] + A[1]*sin((2.0*M_PI*t + M_PI/2)/T + P[1]);
        target[Types::RLEG2]= C[2] + A[2]*sin((2.0*M_PI*t + M_PI/2)/T + P[2]);
        target[Types::RLEG3]= C[3] + A[3]*sin((2.0*M_PI*t + M_PI/2)/T + P[3]);
        target[Types::RLEG4]= C[4] + A[4]*sin((2.0*M_PI*t + M_PI/2)/T + P[4]);
        target[Types::RLEG5]= C[5] + A[5]*sin((2.0*M_PI*t + M_PI/2)/T + P[5]);
        target[Types::RLEG6]= C[6] + A[6]*sin((2.0*M_PI*t + M_PI/2)/T + P[6]);

        return target;
    }
}
#endif