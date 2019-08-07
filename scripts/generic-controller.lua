-- print('hello world')
-- inspect=require'inspect'

log.MELO_INFO_STREAM('HELLO WORLD FROM LUA')
log.MELO_INFO_THROTTLE_STREAM(1.0,'PINGING')

inspect=require'inspect'

print(lcmod,inspect(lcmod))

-- print(lcmod.getName())

function initialize(dt)
  log.MELO_INFO_STREAM('Controller "'..lcmod.getName()..'" is initialized!')
  print('is it true? '..tostring(lcmod.isInitialized()))
  return true
end

function advance(dt)
  return true
end

function reset(dt)
  return true
end

function preStop(dt)
  return true
end

function stop()
  return true
end

function cleanup()
  return true
end

function swap()
  return true
end