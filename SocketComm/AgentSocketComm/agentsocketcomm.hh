/*
 *  Little Green BATS (2008-2013), AI department, University of Groningen
 *
 *  Authors: 	Sander van Dijk (sgdijk@ai.rug.nl)
 *		Drew Noakes (drew@drewnoakes.com)
 *		Martin Klomp (martin@ai.rug.nl)
 *		Mart van de Sanden (vdsanden@ai.rug.nl)
 *		A. Bram Neijt (bneijt@gmail.com)
 *		Matthijs Platje (mplatje@gmail.com)
 *		Jeroen Kuijpers (jkuypers@gmail.com)
 *
 *  Date: 	August 17, 20013
 *
 *  Website:	https://github.com/sgvandijk/libbats
 *
 *  Comment:	Please feel free to contact us if you have any 
 *		problems or questions about the code.
 *
 *
 *  License: 	This program is free software; you can redistribute 
 *		it and/or modify it under the terms of the GNU General
 *		Public License as published by the Free Software 
 *		Foundation; either version 3 of the License, or (at 
 *		your option) any later version.
 *
 *   		This program is distributed in the hope that it will
 *		be useful, but WITHOUT ANY WARRANTY; without even the
 *		implied warranty of MERCHANTABILITY or FITNESS FOR A
 *		PARTICULAR PURPOSE.  See the GNU General Public
 *		License for more details.
 *
 *   		You should have received a copy of the GNU General
 *		Public License along with this program; if not, write
 *		to the Free Software Foundation, Inc., 59 Temple Place - 
 *		Suite 330, Boston, MA  02111-1307, USA.
 *
 */
#ifndef BATS_AGENTSOCKETCOMM_HH
#define BATS_AGENTSOCKETCOMM_HH

#include "../socketcomm.hh"
#include "../../Singleton/singleton.hh"
#include "../../BatsEvent/batsevent.hh"
#include "../../Types/types.hh"
#include <sigc++/sigc++.h>
#include <Eigen/Core>

namespace bats
{
  /**
   * Specialized SocketComm to help an agent communicate with the server
   */
  class AgentSocketComm : public SocketComm
  {
  public:

    /**
     * Update the communication by first sending queued messages and then reading new available input.
     * This method blocks until at least one message is read from the server.
     */
    void update();

    void syn();
    
    static std::shared_ptr<Predicate> makeSynMessage();
    
    /** Send initialization message
     *
     * Send an initialization message to the server, identifying the agent's uniformnumber and teamname
     * @param unum Uniform number (0 = auto choose by server)
     * @param team Team name
     */
    void init(unsigned unum, std::string team);
    
    /** Make an initialization message
     *
     * @param unum Uniform number (0 = auto choose by server)
     * @param team Team name
     */
    static std::shared_ptr<Predicate> makeInitMessage(unsigned unum, std::string team);
    
    /** Move a joint
     *
     * Move a joint by a certain angle. The angle will be added to the current angle of the joint. The move command will be sent the next time update() is called.
     * @param joint The joint to move
     * @param deltaAngle The angle to move the joint by in radians
     */
    void moveJoint(Types::Joint joint, double deltaAngle);
    
    /** Make a command predicate for moving a joint
     *
     * @param joint The joint to move
     * @param deltaAngle The angle to move the joint by in radians
     */
    static std::shared_ptr<Predicate> makeMoveJointMessage(Types::Joint joint, double deltaAngle);
    
    
    /** Move a hinge joint
     *
     * Move a hinge joint by a certain angle. A hinge joint has 1 degree of freedom. The move command will be sent the next time update() is called.
     * @param joint The joint effector name
     * @param deltaAngle The angle to move the joint by in radians
     */
    void moveHingeJoint(Types::Joint joint, double deltaAngle);

    /** Move a hinge torque joint
     *
     * Move a hinge torque joint by applying force.
     * @param joint The joint effector name
     * @param force The force to put on the body parts.
     */
    void moveTorqueJoint(Types::Joint joint, double force);
    
    /** Make a command predicate for moving a hinge joint
     *
     * @param joint The joint effector name
     * @param deltaAngle The angle to move the joint by in radians
     */
    static std::shared_ptr<Predicate> makeMoveHingeJointMessage(Types::Joint joint, double deltaAngle);

    /** Make a command predicate for moving a hinge torque joint
     *
     * @param joint The joint effector name
     * @param force The force to put on te body parts
     */
    static std::shared_ptr<Predicate> makeMoveTorqueJointMessage(Types::Joint joint, double force);
    
    /** Move a universal joint
     *
     * Move a universal joint by a certain angle. A universal joint has 2 degrees of freedom. The move command will be sent the next time update() is called.
     * @param joint The joint effector name (the name of the first axis, eg. Types::LARM1 for lae1_2)
     * @param deltaAngle1 The angle to move the joint by along the first axis in radians
     * @param deltaAngle2 The angle to move the joint by along the second axis in radians
     */
    void moveUniversalJoint(Types::Joint joint, double deltaAngle1, double deltaAngle2);

    /** Make a command predicate for moving a universal joint
     *
     * @param joint The joint effector name (the name of the first axis, eg. Types::LARM1 for lae1_2)
     * @param deltaAngle1 The angle to move the joint by along the first axis in radians
     * @param deltaAngle2 The angle to move the joint by along the second axis in radians
     */
    static std::shared_ptr<Predicate> makeMoveUniversalJointMessage(Types::Joint joint, double deltaAngle1, double deltaAngle2);
    
    /** Beam
     *
     * Beam the agent to a certain position. This command can only be used before kickoff and when a goal is scored and an agent can only beam to his own side of the field. The beam command will be sent the next time update() is called.
     * @param pos The position to beam to. This should be on the own half of the field (x < 0)
     */
    void beam(Eigen::Vector3d const& pos);
    
    /** Make a command predicate for beaming to a certain position
     *
     * @param pos The position to beam to. This should be on the own half of the field (x < 0)
     */
    static std::shared_ptr<Predicate> makeBeamMessage(Eigen::Vector3d const& pos);
    
    /** Say something
     *
     * Shout a short message to team mates. See the documentation of the server for restrictions on message length and shouting distance. The say command will be sent next time update() is called.
     * @param message The message to shout
     */
    void say(std::string message);

    /** Make a command predicate for beaming to a certain position
     *
     * @param message The message to shout
     */
    static std::shared_ptr<Predicate> makeSayMessage(std::string message);

    /** Signal that is fired when a beam message is sent
     */
    sigc::signal<void, std::shared_ptr<BeamEvent> > beam_signal;

  private:
    friend class Singleton<AgentSocketComm>;
    
    AgentSocketComm(AgentSocketComm const&); //NI
    AgentSocketComm& operator=(AgentSocketComm const&); //NI
    
    AgentSocketComm()
    : SocketComm()
    {}
    

  };
  
  typedef Singleton<AgentSocketComm> SAgentSocketComm;
};

#endif
