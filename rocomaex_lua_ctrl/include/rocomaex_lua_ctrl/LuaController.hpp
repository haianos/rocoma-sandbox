/*!
* @file 	  LuaController.hpp
* @author   Enea Scioni
* @date		  09/08/2019
* @version 	1.0
* @brief    Lua Controller Example for rocoma
*/

#pragma once

// roco_ros
#include "roco_ros/controllers/controllers.hpp"

// state and command
#include "rocomaex_lua_ctrl/RocoState.hpp"
#include "rocomaex_model/RocoCommand.hpp"

// std_mmsgs/Float64
#include <std_msgs/Float64.h>

#include <lua.hpp>

namespace rocomaex_lua_ctrl {

class LuaController : 
  virtual public roco_ros::ControllerRos<rocomaex_model::RocoState, rocomaex_model::RocoCommand> {

 public:
  typedef roco_ros::ControllerRos<rocomaex_model::RocoState, rocomaex_model::RocoCommand> Base;
  typedef std::shared_ptr<Base> BasePtr;

  //! Construct LuaController.
  LuaController();

  //! Destruct LuaController.
  virtual ~LuaController();
  
  //! Send Cmd (ROS)
  void sendCmd(const std_msgs::Float64& msg);

 protected:
  //! Create controller LuaController.
  virtual bool create(double dt);

  //! Initialize controller LuaController.
  virtual bool initialize(double dt);

  //! Advance controller LuaController.
  virtual bool advance(double dt);

  //! Reset controller LuaController.
  virtual bool reset(double dt);

  //! Pre-stop controller LuaController.
  virtual bool preStop();

  //! Stop controller LuaController.
  virtual bool stop();

  //! Cleanup controller LuaController.
  virtual bool cleanup();

  //! Swap to controller LuaController with state 'swap'.
  virtual bool swap(double dt, const roco::ControllerSwapStateInterfacePtr& swapState);

  //! Get swap state 'swapState' of controller LuaController.
  virtual bool getSwapState(roco::ControllerSwapStateInterfacePtr& swapState);

  //! Add shared module 'module' to controller LuaController.
  virtual bool addSharedModule(const roco::SharedModulePtr& module);

 private:
  lua_State* L;
  void initState();
  int  set_ctrl();
  int  executeFile(const std::string &name);
  int  executeString(const std::string &str);
  bool callControllerStep(const std::string &stepname);
  bool callControllerStep(const std::string &stepname, double dt);
  ros::Publisher _pub_cmd;
  std::string _path; //internal path to controllers folder
};

} /* namespace rocomaex_lua_ctrl */
