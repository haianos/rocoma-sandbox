/*!
* @file 	  rocomaex_lua_cltr_plugin.cpp
* @author   Enea Scioni
* @date		  09/09/2019
* @version 	0.1
* @brief    Plugin export for controller LuaController
*/

// state and command
#include "rocomaex_model/RocoState.hpp"
#include "rocomaex_model/RocoCommand.hpp"

// rocomaex_ros_ctrl1
#include "rocomaex_lua_ctrl/LuaController.hpp"

// rocoma_plugin
#include "rocoma_plugin/rocoma_plugin.hpp"

// export controller plugin
ROCOMA_EXPORT_CONTROLLER_ROS(
  LuaController,
  rocomaex_model::RocoState,
  rocomaex_model::RocoCommand,
  rocomaex_lua_ctrl::LuaController
);
