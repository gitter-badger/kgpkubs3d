#ifndef __INVERSEKINEMATICS_HH_
#define __INVERSEKINEMATICS_HH_

#include <vector>
#include <map>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <queue>
#include <stack>
#include <cmath>
#include <list>

#include "../../Distribution/NormalDistribution/normaldistribution.hh"
#include "../../Singleton/singleton.hh"
#include "../../Types/types.hh"
#include "../../Math/math.hh"
#include "../../AgentModel/agentmodel.hh"

using namespace std;
using namespace Eigen;

namespace bats
{
    
    const double MINDIST            = 0.15;
    const double MINANGLEDIFF   = M_PI/9;

    class InverseKinematics
    {
    public:
    /*
     * Left hand    : -0.098,0.14,0.084
     * Right hand   :  0.098,0.14,0.084
     * Right Leg    :  0.055 0.025 -0.37
     * Left Leg     : -0.055 0.025 -0.37
     * Head             :
     * */
        InverseKinematics(AgentModel& am_,
                          Cochlea& cochlea_,
                          AgentSocketComm& comm_,
                          Cerebellum& cer_,
                          WorldModel& wm_,
                          Localizer& loc_,
                          Types::IKMove move):am(am_),cochlea(cochlea_),comm(comm_),cer(cer_),wm(wm_),loc(loc_) {

            switch(move) {
            case Types::LEFTLEG:
                cout << "Left Leg\n";
                start_link = am.getJoint(Types::LLEG1);
                break;

            case Types::RIGHTLEG:
                cout << "Right Leg\n";
                start_link= am.getJoint(Types::RLEG1);
                break;

            case Types::LEFTHAND:
                cout << "Left Hand\n";
                start_link= am.getJoint(Types::LARM1);
                break;

            case Types::RIGHTHAND:
                cout << "Right Hand\n";
                start_link = am.getJoint(Types::RARM1);
                break;

            case Types::FACE:
                cout << "Face\n";
                start_link = am.getJoint(Types::HEAD1);
                break;
            }

            eLimb=dynamic_pointer_cast<Limb>(getEffector(start_link));
            cout << "Efector Name: " << eLimb->name << endl;
        }

        // This fucntion takes
        void run(const Vector3d Target);

    private:

        bool nextIK();

        void stopIK();

        bool comIK();

        double contraints(Types::Joint,double initial,double change);

        // Updates the joints and limbs stransform function after every iteration
        void updatePosture(shared_ptr<BodyPart> bp,double angle);

        shared_ptr<BodyPart> getEffector(shared_ptr<BodyPart> bp);

        AgentModel& am;
        Cochlea& cochlea;
        AgentSocketComm& comm;
        Cerebellum& cer;
        WorldModel& wm;
        Localizer& loc;

        shared_ptr< BodyPart > present_link;

        shared_ptr< BodyPart > start_link;

        queue < shared_ptr<BodyPart> > bodyQueue;

        shared_ptr<Limb> eLimb;

        Vector3d Target;

        map< Types::Joint , double > initial;
        map< Types::Joint , double > change;

        Types::IKState ikState;

    };

    void InverseKinematics::run(const Vector3d target_)
    {
        ikState = Types::NEXTPART;
        Target = target_;
        bodyQueue.push(start_link);

        while(1) {
            cout << "Updating Environment\n";
            comm.update();
            cochlea.update();
            wm.update();
            am.update();
            loc.update();

            cout << "Checking IK\n";

            switch(ikState) {
            case Types::NEXTPART:
                cout << "Next Part\n";

                if(!nextIK())
                    ikState = Types::MOVING;
                else
                    ikState = Types::ENDIK;

                break;

            case Types::MOVING:
                cout << "Moving\n";

                if(comIK())
                    ikState = Types::STOP;

                break;

            case Types::STOP:
                cout << "Stop\n";
                stopIK();
                ikState = Types::NEXTPART;
                break;

            case Types::ENDIK:
                cout << "EndIK\n";
                stopIK();
                return;
            }

            cout << "Cycle Complete\n";
        }
    }

    bool InverseKinematics::nextIK()
    {
        // Get actual body position.

        // Map for storing angles
        double theta,dist;

        Vector3d projT,projE,Tl,El,jPos,axisL,Effector,AEffector;

        Effector = eLimb->transform.translation();
        cout << "Effector Position: " << Effector.transpose() << " Target Position: " << Target.transpose() << endl;
        dist = Math::distance(Effector,Target);
        cout << "Distance(Effector,Target): " << dist << endl;

        if( dist  < MINDIST)
            return true;

        //////Main algorithm//////////////////////////////////////////////////////////////
        cout << "Iteration Started\n";
        present_link = bodyQueue.front();
        bodyQueue.pop();

        if(shared_ptr<Limb> l = dynamic_pointer_cast<Limb>(present_link) ) {
            cout << "Pushing Joints\n";

            // Push all joints linked to this body part into the queue
            if(l->joints.empty()) {
                cout << "Pushing Start Link\n";
                bodyQueue.push(start_link);
            }

            for(vector<shared_ptr<Joint> >::iterator it = l->joints.begin(); it!=l->joints.end(); it++)
                bodyQueue.push(*it);
        }
        else {
            shared_ptr<Joint> joint = dynamic_pointer_cast<Joint>(present_link);
            bodyQueue.push(joint->bodyPart);
            
            // Calculate point with respec to link
            jPos = joint->transform.translation();//Position with respect to torso
            Tl = Target - jPos; // Target with respect to link
            El = Effector - jPos;// Effector with repect to link

            // Calcualate projection in plane of link.
            Vector3d lAV = joint->getAxisVec();

            projT = Tl - lAV.dot(Tl)*lAV/lAV.norm();

            projE = El - lAV.dot(El)*lAV/lAV.norm();


            // Calculate the theta required to move the effector
            // The line joining the link to target.
            theta = atan2 (Math::signum(projE.cross(projT).dot(lAV))*projE.cross(projT).norm(),projT.dot(projE));
            theta = contraints(joint->id,joint->angle->getMu()(0),theta);
            cout << joint->name << endl;
            cout << "Axis Vector: " << lAV.transpose() << endl;
            cout << "El: " << El.transpose() << " Tl: " << Tl.transpose() << endl;
            cout << "PE: " << projE.transpose() << " PT: " << projT.transpose() << endl;
            //cout <<j->id << " theta: " << theta  << endl;
            if(projE.norm()<0.01 || projT.norm()<0.01)
                theta = 0;
            change[joint->id] = theta;
            //getchar();
            /*
                        //Update Posture
                        updatePosture(j, angles[j->id] );
                        Effector = eLimb->transform.translation();
                        //Close enough
                        dist = Math::distance(Effector,Target);
                        cout << "Effector Position: " << Effector.transpose() << " Target Position: " << Target.transpose() << endl;
                        cout << "Distance(Effector,Target): " << dist << endl;
                        //getchar();
            */
        }

        /////////////////////////////////////////////////////////////////////////////////////
        return false;
    }


    void InverseKinematics::updatePosture( shared_ptr< BodyPart> bp,double turn_angle)
    {
        list<shared_ptr<BodyPart> > partQueue;
        partQueue.push_back(bp);
        double a;

        while (!partQueue.empty()) {
            shared_ptr<BodyPart> part = partQueue.front();
            partQueue.pop_front();

            if (shared_ptr<Limb> limb = dynamic_pointer_cast<Limb>(part)) {
                //_debugLevel4(limb->name << " " << limb->transform.translation() << " " << limb->relativeWeight);
                for (vector<shared_ptr<Joint> >::iterator iter = limb->joints.begin(); iter != limb->joints.end(); ++iter) {
                    shared_ptr<Joint> joint = *iter;
                    //_debugLevel4(joint->name << " " << joint->anchors.first);
                    joint->transform = limb->transform *
                                       Translation3d(joint->anchors.first);

                    partQueue.push_back(joint);
                }
            }
            else {
                shared_ptr<Joint> joint = dynamic_pointer_cast<Joint>(part);
                //_debugLevel4(joint->name << " " << joint->transform.translation() << " " << angle);

                if (joint->name == bp->name) {
                    a = joint->angle->getMu()(0)+turn_angle;
                    cout << joint->name << " Angle: " << (180*a/M_PI) << " axis: " << joint->axis.transpose() << endl;
                    joint->angle->init(VectorXd::Constant(1,a), VectorXd::Constant(1,1.0));
                }

                //_debugLevel4(joint->name << " " << -joint->anchors.second);

                shared_ptr<BodyPart> part2 = joint->bodyPart;
                part2->transform = joint->transform *
                                   AngleAxisd(joint->angle->getMu()(0), joint->axis) *
                                   Translation3d(-joint->anchors.second);
                //_debugLevel4(part2->transform);
                partQueue.push_back(part2);
            }
        }
    }

    shared_ptr<BodyPart> InverseKinematics::getEffector(shared_ptr<BodyPart> bp)
    {
        queue< shared_ptr<BodyPart> > q;
        shared_ptr<BodyPart> bpThis;
        q.push(bp);

        while(1) {
            bpThis = q.front();
            q.pop();

            if(shared_ptr<Limb> l = dynamic_pointer_cast<Limb>(bpThis) ) {
                if(l->joints.empty()) {
                    q.push(bpThis);
                    break;
                }

                for(vector<shared_ptr<Joint> >::iterator it = l->joints.begin(); it!=l->joints.end(); it++) {
                    q.push(*it);
                }
            }
            else {
                shared_ptr<Joint> j = dynamic_pointer_cast<Joint>(bpThis);
                q.push(j->bodyPart);
            }
        }

        //cout << "Effector: " << q.front()->name << endl;
        return q.front();
    }

    bool InverseKinematics::comIK()
    {
        Vector3d Effector = eLimb->transform.translation();
        double dist = Math::distance(Effector,Target);
        double present;
        double init;
        double chan;
        double velocity;

        cout << "Effector Position: " << Effector.transpose() << " Target Position: " << Target.transpose() << endl;
        cout << "Distance(Effector,Target): " << dist << endl;

        if(shared_ptr<Joint> moveJ = dynamic_pointer_cast<Joint>(present_link)) {
            present = am.getJoint(moveJ->id)->angle->getMu()(0);
            init = initial[moveJ->id];
            chan = change[moveJ->id];
            velocity = Math::signum(chan)*fabs(present-(init+chan));
            cout << moveJ->name << endl;
            cout << "Sending : " << 180*present/M_PI << " " << 180*init/M_PI << " " << 180*chan/M_PI << endl;
            cer.addAction(make_shared<MoveJointAction>( moveJ->id,  velocity) );
            cer.outputCommands(comm);

            if(fabs(present - (init+chan))  > MINANGLEDIFF && dist > MINDIST)
                return false;
        }

        return true;
    }

// All those joints which were given velocity will now get zero velocity so that they stop.
    void InverseKinematics::stopIK()
    {
        for(map< Types::Joint , double >::iterator it=initial.begin(); it!=initial.end(); it++) {
            cer.addAction( make_shared<MoveJointAction>( it->first,0) );
        }

        cer.outputCommands(comm);
        //getchar();
    }

    double InverseKinematics::contraints(Types::Joint joint,double initial,double change)
    {
        double lower;
        double upper;

        switch(joint) {

        case Types::HEAD1:        ///< Neck, Z-Axis (-120, 120)
            lower = -120;
            upper = 120;
            break;

        case Types::HEAD2:   ///< Neck, X-Axis (-45, 45)
            lower = -45;
            upper = 45;
            break;

        case Types::LLEG1:    ///< Left hip, XZ-Axis (-90, 1)
            lower = -90;
            upper = 1;
            break;

        case Types::LLEG2:    ///< Left hip, X-Axis (-25, 100)
            lower = -25;
            upper = 100;
            break;

        case Types::LLEG3:    ///< Left hip, Y-Axis (-25, 45)
            lower = -25;
            upper = 45;
            break;

        case Types::LLEG4:   ///< Left knee, X-Axis (-130, 1)
            lower = -130;
            upper = 1;
            break;

        case Types::LLEG5:    ///< Left ankle, X-Axis (-45, 75)
            lower = -45;
            upper = 75;
            break;

        case Types::LLEG6:    ///< Left ankle, Y-Axis (-25, 45)
            lower = -25;
            upper = 45;
            break;

        case Types::RLEG1:    ///< Right hip, XZ-Axis (-90, 1)
            lower = -90;
            upper = 1;
            break;

        case Types::RLEG2:    ///< Right hip, X-Axis (-25, 100)
            lower = -25;
            upper = 100;
            break;

        case Types::RLEG3:    ///< Right hip, Y-Axis (-45, 25)
            lower = -45;
            upper = 25;
            break;

        case Types::RLEG4:    ///< Right knee, X-Axis (-130, 1)
            lower = -130;
            upper = 1;
            break;

        case Types::RLEG5:    ///< Right ankle, X-Axis (-45, 75)
            lower = -45;
            upper = 75;
            break;

        case Types::RLEG6:    ///< Right ankle, Y-Axis (-25, 45)
            lower = -25;
            upper = 45;
            break;

        case Types::LARM1:    ///< Left shoulder, X-Axis (-120, 120)
            lower = -120;
            upper = 120;
            break;

        case Types::LARM2:    ///< Left shoulder, Z-Axis (-1, 95)
            lower = -1;
            upper = 95;
            break;

        case Types::LARM3:    ///< Left shoulder, Y-Axis (-120, 120)
            lower = -120;
            upper = 120;
            break;

        case Types::LARM4:    ///< Left elbow, Z-Axis (-90, 1)
            lower = -90;
            upper = 1;
            break;

        case Types::RARM1:    ///< Right shoulder, X-Axis (-120, 120)
            lower = -120;
            upper = 120;
            break;

        case Types::RARM2:    ///< Right shoulder, Z-Axis (-95, 1)
            lower = -95;
            upper = 1;
            break;

        case Types::RARM3:    ///< Right shoulder, Y-Axis (-120, 120)
            lower = -120;
            upper = 120;
            break;

        case Types::RARM4:     ///< Right elbow, Z-Axis (-1, 90)
            lower = -1;
            upper = 90;
            break;
        }

        lower *= M_PI/180;
        upper *= M_PI/180;
        double angle = initial + change;
        cout << "Before Const: " << 180*angle/M_PI << " " << 180*initial/M_PI << " " << 180*change/M_PI << endl;
        angle = max(lower,angle);
        angle = min(upper,angle);
        change = angle - initial;
        cout << "After Const: " << 180*angle/M_PI << " " << 180*initial/M_PI << " " << 180*change/M_PI << endl;
        return change;
    }
}
#endif