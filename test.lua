app = helo.heloApp.theApp
q = app:getEventQueue()
cam = app:getCamera()
cam:setTrackable(nil)

function printVector (v)
  print(v.x .. ", " .. v.y .. ", " .. v.z)
end


curp = cam:getPosition()
printVector(curp)
curd = cam:getDirection()
printVector(curd)

cam:setPosition(helo.Vector3(curp.x, curp.y + 5, curp.z))
cam:setLookAt(helo.Vector3(curp.x, curp.y, curp.z + 1))

conf = app:getConfiguration()
controllables = conf:getControllables()
c = controllables[0]
t = c:toTrackable()
cam:setTrackable(t)

phys = app:getPhysics()
v = conf:loadVehicle("HMMWV", "hmmwv.1", helo.Vector3(1780.96, 2.0, 1635.3), helo.Vector3(0, 190, 0))
po = v:toPhysicsObject()
po:finishPhysicsConfiguration(phys)

function make_printer(s)
  local out = "make_printer output: " .. s
  return function()
  	   print(out)
	 end
end

cb1 = helo.postEventToQueue(q, 10000, function ()
                                 print("Callback at 100 in lua!!!")
	                       end)

cb2 = helo.postEventToQueue(q, 50000, function ()
                                 print("Callback at 50 in lua!!!")
	                       end)

print("Callbacks: " .. cb1 .. " and " .. cb2)


helo.postEventToQueue(q, 1500000, make_printer(75))
helo.postEventToQueue(q, 75000, make_printer("Closure at 75 ms"))

function repostable_event(actual_time, event_time, id)
  print("Repeat callback " .. id .. " at time " .. event_time .. " (+" .. actual_time - event_time .. ")")
  helo.postEventToQueue(q, event_time + 5000000, repostable_event)
end

helo.postEventToQueue(q, 5000000, repostable_event)
