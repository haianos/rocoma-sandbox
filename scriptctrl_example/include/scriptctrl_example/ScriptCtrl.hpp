/*
 * Enea Scioni, <enea.scioni@gmail.com>
 * Proof-of-concept of extension rocoma ControllerManagerRos with Lua scripting
 * 
 * based on rocoma_ros_example
 * 
 * 09/09/2019
 */

#pragma once

// any_node
#include "any_node/Node.hpp"

// rocoma ros
#include "rocoma_ros/ControllerManagerRos.hpp"

// rocomaex model
#include "rocomaex_model/RocoState.hpp"
#include "rocomaex_model/RocoCommand.hpp"


//#include <lua.hpp>
#include <scriptctrl_example/lcm.hpp>

namespace scriptctrl_example {

/** This example shows how to setup a controller manager ros. To interact with the manager use the ros services
 *  provided by rocoma_ros::ControllerManagerRos.
 */
class ScriptCtrl: public any_node::Node {
public:
  //! Delete default constructor
  ScriptCtrl() = delete;

  //! Constructor
  ScriptCtrl(NodeHandlePtr nodeHandle);

  //! Destructor
  ~ScriptCtrl() override = default;

  //! Init the ros example
  bool init() override;

  //! Update the ros example
  bool update(const any_worker::WorkerEvent& event);

  //! Cleanup the ros example
  void cleanup() override;

 private:
  //! Time step determines update frequency
  double timeStep_;

  //! Controller manager ros
  rocoma_ros::ControllerManagerRos<rocomaex_model::RocoState, rocomaex_model::RocoCommand> controllerManager_;

  //! Robot state
  std::shared_ptr<rocomaex_model::RocoState> state_;

  //! Actuator commands
  std::shared_ptr<rocomaex_model::RocoCommand> command_;

  //! Mutex for robot state
  std::shared_ptr<boost::shared_mutex> mutexState_;

  //! Mutex for actuator commands
  std::shared_ptr<boost::shared_mutex> mutexCommand_;
  
  //! Lua Scripting state
  lua_State* L;
  void set_lstate();
  bool repl(const any_worker::WorkerEvent& event);
  int  executeFile(const std::string &name);
};

}
