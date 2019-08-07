/*!
* @file 	  LuaController.hpp
* @author   Enea Scioni
* @date		  09/09/2019
* @version 	1.0
* @brief    Lua Controller Example for rocoma
*/

#pragma once

// roco_ros
#include "roco_ros/controllers/controllers.hpp"

// state and command
#include "rocomaex_model/RocoState.hpp"
#include "rocomaex_model/RocoCommand.hpp"

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
  int  set_ctrl(); //LuaController* lc);
  int  executeFile(const std::string &name);
  int  executeString(const std::string &str);
  bool callControllerStep(const std::string &stepname);
  bool callControllerStep(const std::string &stepname, double dt);
};

} /* namespace rocomaex_lua_ctrl */
