#include "scriptctrl_example/lcm.hpp"
#define luaM_checkudata_bx(L, pos, T) (T**) (luaL_checkudata((L), (pos), #T))

static int getCMR(lua_State *L)
{
	lua_pushstring(L, "this_CMR");
	lua_rawget(L, LUA_REGISTRYINDEX);
	return 1;
}


static int Controller_getActiveControllerName(lua_State *L) {
  CMR *ctrl;
  getCMR(L);
  ctrl = *(luaM_checkudata_bx(L, -1, CMR));
  lua_pushstring(L,ctrl->getActiveControllerName().c_str());
  return 1;
}

static int Controller_printActive(lua_State *L) {
  CMR *ctrl;
  getCMR(L);
  ctrl = *(luaM_checkudata_bx(L, -1, CMR));
  std::cout << ctrl->getActiveControllerName() << std::endl;
  return 1;
}

static int Controller_switchController(lua_State* L) {
  int n = lua_gettop(L);
  if ( n != 1) {
    lua_pushstring(L,"calling switchController with incorrect number of args");
    lua_error(L);
  }
  std::string s(luaL_checkstring(L,1));
  CMR *ctrl;
  getCMR(L);
  ctrl = *(luaM_checkudata_bx(L, -1, CMR));
  int ret = static_cast<int>(ctrl->switchController(s));
  switch(ret){
    case -2 : {
      lua_pushstring(L,"NOTFOUND");
      break;
    }
    case -1 : {
      lua_pushstring(L,"ERROR");
      break;
    }
    case 0 : {
      lua_pushstring(L,"NA");
      break;
    }
    case 1 : {
      lua_pushstring(L,"RUNNING");
      break;
    }
    case 2 : {
      lua_pushstring(L,"SWITCHING");
      break;
    }
    default :
      lua_pushstring(L,"UNKNOWN");
      break;
  }
  return 1;
}

static int Controller_getAvailableControllers(lua_State* L) {
  CMR *ctrl;
  getCMR(L);
  ctrl = *(luaM_checkudata_bx(L, -1, CMR));
  std::vector<std::string> ret = ctrl->getAvailableControllerNames();
  lua_createtable(L,ret.size(),0);
  for(unsigned int i=0;i<ret.size();i++) {
    lua_pushinteger(L,i+1);
    lua_pushstring(L,ret[i].c_str());
    lua_settable(L,-3);
  }
  return 1;
}

static int Controller_printAvailableControllers(lua_State* L) {
  CMR *ctrl;
  getCMR(L);
  ctrl = *(luaM_checkudata_bx(L, -1, CMR));
  std::vector<std::string> ret = ctrl->getAvailableControllerNames();
  lua_createtable(L,ret.size(),0);
  for(unsigned int i=0;i<ret.size();i++) 
    std::cout << ret[i] << std::endl;
  return 1;
}

static const struct luaL_Reg ctrlmodreg[] = {
  {"getActiveControllerName",Controller_getActiveControllerName},
  {"switch",Controller_switchController},
  {"getControllers",Controller_getAvailableControllers},
  {"printControllers",Controller_printAvailableControllers},
  {"printActive",Controller_printActive},
  {NULL,NULL}
};

int luaopen_ctrlmodule(lua_State *L) {
  luaL_newmetatable(L,"CMR");
  luaL_newlib(L,ctrlmodreg);
  return 1;
}