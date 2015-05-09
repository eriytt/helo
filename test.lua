app = helo.heloApp.theApp
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
c = controllables[1]
t = c:toTrackable()
cam:setTrackable(t)

