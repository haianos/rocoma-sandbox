/*
 * Enea Scioni, <enea.scioni@gmail.com>
 * 09/09/2019
 * 
 * Lua wrapping of finalised class CMR (ControllerManagerRos<RocoState,RocoCommand>
 */

#pragma once

#include <lua.hpp>
#include <rocoma_ros/ControllerManagerRos.hpp>

// rocomaex model
#include <rocomaex_model/RocoState.hpp>
#include <rocomaex_model/RocoCommand.hpp>

/* Those are from ControllerManager Class */
using namespace rocoma_ros;
using namespace rocomaex_model;

typedef ControllerManagerRos<RocoState,RocoCommand> CMR;

int luaopen_ctrlmodule(lua_State *L);