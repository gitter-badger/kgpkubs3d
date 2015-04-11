
#ifndef CoreAGENT_HH
#define CoreAGENT_HH

#include <HumanoidAgent/humanoidagent.hh>
#include <JointController/GaitGenerator/IKGaitGenerator/ikgaitgenerator.hh>
#include <JointController/MotionSequencePlayer/motionsequenceplayer.hh>
#include <map>

/** A dribbling robot
  */
class CoreAgent : public bats::HumanoidAgent
{
    public:

    /** The Constructor
     *
     *  Sets this agent's teamname to "Hello". Consider putting initialization stuff in init() instead of here.
     */
    CoreAgent();

    const int MacroGoalieLimit=-12.0;
    const int MacroDefenderLimit=-7.0;
    const int MacroMidFielderLimit=3.0;
    private:
    // The agent's gait generator
    std::shared_ptr<bats::GaitGenerator> d_gaitGenerator;

    // Motion sequence players
    std::map<std::string, std::shared_ptr<bats::MotionSequencePlayer>> d_motionSequencePlayers;

    // Used to smoothen walking parameters
    Eigen::VectorXd d_paramFilter;

    // Flag to remember that we have beamed
    bool d_beamed;

    // Flag to remember we are getting up
    int d_gettingUpFrom;



    /** Initialize agent
     *
     * Called a single time when starting up the agent. Put all your initialization stuff here.
     */
    virtual void init();

    /** Think cycle
     *
     * Called at each cycle after a message from the server is received and parsed. Put all your thinking and acting stuff here.
     */
    virtual void think();

    /** Determine joint velocities to move into a standing position
     */
    Eigen::VectorXd stand();

    /** Determine joint angles for head joints in order to look at the ball
     */
    Eigen::Vector2d determineWhereToLook();

    /** Determine walking and facing directions
     */
    Eigen::VectorXd determineWhereToWalk();
    void goToPoint(const Eigen::Vector3d&);
    void takeBallToGoal();
    void fallNow(int sideToFall);


};

#endif
