--[[
    Enea Scioni, <enea.scioni@gmail.com>
    09/08/2019
   Example of Controller written in Lua
--]]

function initialize(dt)
  log.MELO_INFO_STREAM('Controller "'..lcmod.getName()..'" is initialized!')
  local state = lcmod.getState()
  state:setValue(0.0)
  return true
end

function advance(dt)
  log.MELO_INFO_STREAM('Controller "'..lcmod.getName()..'": update (dt='..tostring(dt)..')')
  local k = 0.5;
  command = lcmod.getCommand()
  state   = lcmod.getState()
  command:setValue(state:getValue()+(k*dt))
  -- send command to ROS topic as well
  lcmod.pubCmd(command:getValue())
  -- emulate ideal case
  state:setValue(command:getValue())
  return true
end

function reset(dt)
  log.MELO_INFO_STREAM('Controller "'..lcmod.getName()..'": reset (dt='..tostring(dt)..')')
  return initialize(dt)
end

function preStop()
  log.MELO_INFO_STREAM('Controller "'..lcmod.getName()..'": pre-stop')
  return true
end

function stop()
  log.MELO_INFO_STREAM('Controller "'..lcmod.getName()..'": stop')
  return true
end

function cleanup()
  log.MELO_INFO_STREAM('Controller "'..lcmod.getName()..'": cleaup')
  return true
end

function swap(dt)
  if lcmod.isInitialized() then
    return reset(dt)
  else
    return initialize(dt)
  end
  return true
end