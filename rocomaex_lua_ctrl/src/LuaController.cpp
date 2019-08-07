/*!
* @file 	  LuaController.cpp
* @author   Enea Scioni
* @date		  09/09/2019
* @version 	0.1
* @brief    Lua Controller Example for rocoma
*/

// rocomaex_lua_ctrl
#include "rocomaex_lua_ctrl/LuaController.hpp"

// logger
#include <message_logger/message_logger.hpp>

#define luaM_checkudata_bx(L, pos, T) (T**) (luaL_checkudata((L), (pos), #T))

#define get_lua_cltr \
  LuaController *lc; \
  getCtrl(L); \
  lc = *(luaM_checkudata_bx(L, -1, LuaController)); \
  
namespace rocomaex_lua_ctrl {

LuaController::~LuaController() {
  lua_close(L);
}

bool LuaController::create(double dt) {
  MELO_INFO_STREAM("Controller " << this->getName() << " is created!");
  return true;
}

bool LuaController::initialize(double dt) {
  std::cout << "PARAM PATH is: " << this->getParameterPath() << std::endl;
  this->executeFile("/home/haianos/anybotics/generic-controller.lua");
  callControllerStep("initialize");
  return true;
}

bool LuaController::advance(double dt) {
  MELO_INFO_THROTTLE_STREAM(1.0, "Controller " << this->getName() << " is advanced!");
  return true;
}

bool LuaController::reset(double dt) {
  MELO_INFO_STREAM("Controller " << this->getName() << " is reset!");
  return LuaController::initialize(dt);
}

bool LuaController::preStop() {
  MELO_INFO_STREAM("Controller " << this->getName() << " is pre-stopped!");
  return true;
}

bool LuaController::stop() {
  MELO_INFO_STREAM("Controller " << this->getName() << " is stopped!");
  return true;
}

bool LuaController::cleanup() {
  MELO_INFO_STREAM("Controller " << this->getName() << " is cleaned up!");
  return true;
}

bool LuaController::swap(double dt, const roco::ControllerSwapStateInterfacePtr& swapState) {
  // Call current class reset / initialize
  return isInitialized() ? LuaController::reset(dt) : LuaController::initialize(dt);
}

bool LuaController::getSwapState(roco::ControllerSwapStateInterfacePtr& swapState) {
  swapState.reset(nullptr);
  return true;
}

bool LuaController::addSharedModule(const roco::SharedModulePtr& module) {
  return false;
}

int LuaController::executeFile(const std::string &name) {
  int error = luaL_loadfile(L, name.c_str());
  if (error != 0) {
    std::cerr << "executeFile : error during loading/compiling :\n "
              << lua_tostring(L, -1) << std::endl
              << std::endl;
    lua_pop(L, 1); /* pop error message from the stack */
    return 1;
  }
  error = lua_pcall(L, 0, 0, 0);
  if (error != 0) {
    std::cerr << "executeFile : error during execution : \n"
              << lua_tostring(L, -1) << std::endl
              << std::endl;
    lua_pop(L, 1); /* pop error message from the stack */
    return 2;
  }
  return 0;
}

int LuaController::executeString(const std::string &str) {
  const char *buf = str.c_str();
  int error = luaL_loadbuffer(L, buf, str.length(), "executeString");
  if (error != 0) {
    std::cerr << "executeString : error during loading/compiling :\n "
              << lua_tostring(L, -1) << std::endl;
    lua_pop(L, 1); /* pop error message from the stack */
    return 1;
  }
  error = lua_pcall(L, 0, 0, 0);
  if (error != 0) {
    std::cerr << "executeString : error during execution :\n "
              << lua_tostring(L, -1) << std::endl;
    lua_pop(L, 1); /* pop error message from the stack */
    return 2;
  }
  return 0;
}

bool LuaController::callControllerStep(const std::string& stepname) {
  MELO_INFO_STREAM("STACK (entry)" << lua_gettop(L));
  lua_getglobal(L,stepname.c_str());
  int error = lua_pcall(L,0,1,0);
  if (error != 0) {
    MELO_ERROR_STREAM("Error executing LuaController(" << this->getName() << ") -> " 
    << stepname << ": "<< lua_tostring(L,-1));
    lua_pop(L,2);
    return false;
  }
  if (0==lua_toboolean(L,-1))
    return false;
  lua_pop(L,1);
  MELO_INFO_STREAM("STACK (exit)" << lua_gettop(L));
  return true;
}

bool LuaController::callControllerStep(const std::string& stepname,double dt) {
  MELO_INFO_STREAM("STACK (entry)" << lua_gettop(L));
  lua_getglobal(L,stepname.c_str());
  lua_pushnumber(L,dt);
  int error = lua_pcall(L,1,1,0);
  if (error != 0) {
    MELO_ERROR_STREAM("Error executing LuaController(" << this->getName() << ") -> " 
    << stepname << ": "<< lua_tostring(L,-1));
    lua_pop(L,2);
    return false;
  }
  if (0==lua_toboolean(L,-1))
    return false;
  MELO_INFO_STREAM("STACK (exit)" << lua_gettop(L));
  return true;
  
}

int LuaController::set_ctrl() { //LuaController* lc) {
    LuaController **new_lc;
    lua_pushstring(L,"this_LC");
    new_lc  = (LuaController**) lua_newuserdata(L, sizeof(LuaController*));
    *new_lc = (LuaController*) (this);
    luaL_getmetatable(L, "LuaController");
    lua_setmetatable(L, -2);
    lua_rawset(L, LUA_REGISTRYINDEX);
    return 0;
};

} /* namespace rocomaex_lua_ctrl */

/* set log defines*/

static int melo_error_stream(lua_State *L) {
  int n = lua_gettop(L);
  if ( n!=1 or (not lua_isstring(L,1))) {
    lua_pushstring(L,"calling melo_error_stream with incorrect number of args");
    lua_error(L);
  }
  MELO_ERROR_STREAM(lua_tostring(L,1));
  return 0;
}

static int melo_info_stream(lua_State *L) {
  int n = lua_gettop(L);
  if ( n!=1 or (not lua_isstring(L,1))) {
    lua_pushstring(L,"calling melo_info_stream with incorrect number of args");
    lua_error(L);
  }
  MELO_INFO_STREAM(lua_tostring(L,1));
  return 0;
}

static int melo_info_throttle_stream(lua_State *L) {
  int n = lua_gettop(L);
  if (n != 2) {
    lua_pushstring(L,"calling MELO_INFO_THROTTLE_STREAM with incorrect number of args");
    lua_error(L);
  }
  if ( (not lua_isstring(L,2)) or (not lua_isnumber(L,1)) ) {
    lua_pushstring(L,"calling MELO_INFO_THROTTLE_STREAM with wrong type of args");
    lua_error(L);
  }
  MELO_INFO_THROTTLE_STREAM(lua_tonumber(L,1), lua_tostring(L,2));
  return 0;
}

static const struct luaL_Reg logreg[] = {
 {"MELO_INFO_STREAM",  melo_info_stream},
 {"MELO_ERROR_STREAM", melo_error_stream},
 {"MELO_INFO_THROTTLE_STREAM", melo_info_throttle_stream},
 {NULL, NULL}
};

// gen_push_bxptr(TaskContext_push, "TaskContext", TaskContext)

static int getCtrl(lua_State *L)
{
	lua_pushstring(L, "this_LC");
	lua_rawget(L, LUA_REGISTRYINDEX);
	return 1;
}

using namespace rocomaex_lua_ctrl;

static int LuaController_getName(lua_State *L) {
  const char *s;
  LuaController *lc;
  getCtrl(L);
  lc = *(luaM_checkudata_bx(L, -1, LuaController));
  s = lc->getName().c_str();
  lua_pushstring(L,s);
  return 1;
}

static int LuaController_isInitialized(lua_State *L) {
  LuaController *lc;
  getCtrl(L);
  lc = *(luaM_checkudata_bx(L, -1, LuaController));
  bool ret = lc->isInitialized();
  lua_pushboolean(L,ret ? 1 : 0 );
  return 1;
}

static const struct luaL_Reg lcmodreg[] = {
  {"isInitialized", LuaController_isInitialized},
//   {"getName", LuaController_isCreated},
//   {"getName", LuaController_isRunning},
  {"getName", LuaController_getName},
  {"getCtrl", getCtrl},
  {NULL, NULL}
};

int luaopen_logmodule(lua_State *L) {
  luaL_newlib(L,logreg);
  return 1;
}

int luaopen_lcmodule(lua_State *L) {
  luaL_newmetatable(L,"LuaController");
  //luaL_setfuncs (L, lcmodreg, 0);
  luaL_newlib(L,lcmodreg);
  return 1;
}


// /* Those are from ControllerManager Class */
// 
// static int getCM(lua_State *L)
// {
// 	lua_pushstring(L, "this_CM");
// 	lua_rawget(L, LUA_REGISTRYINDEX);
// 	return 1;
// }
// 
// 
// static int Controller_getActiveControllerName(lua_State *L) {
//   LuaController::Base *ctrl;
//   getCM(L);
//   ctrl = *(luaM_checkudata_bx(L, -1, LuaController::Base));
// //   std::cout << ctrl->getActiveControllerName() << std::endl;
// //   std::string str(lc->getActiveControllerName());
// //   lua_pushboolean(L,ret ? 1 : 0 );
//   return 1;
// }
// 
// static const struct luaL_Reg ctrlmodreg[] = {
//   {"getActiveControllerName",Controller_getActiveControllerName},
//   {NULL,NULL}
// };
// 
// int luaopen_ctrlmodule(lua_State *L) {
//   luaL_newmetatable(L,"LuaController::Base");
//   luaL_newlib(L,ctrlmodreg);
//   return 1;
// }


namespace rocomaex_lua_ctrl {
  
  LuaController::LuaController()
    : Base()
  {
    setName("LuaController");
    initState();
    set_ctrl();
  }
   
  void LuaController::initState() {
    L = luaL_newstate();
    luaL_openlibs(L);
    
    luaopen_logmodule(L);
    lua_setglobal(L,"log");
    luaopen_lcmodule(L);
    lua_setglobal(L,"lcmod");
  }

}
