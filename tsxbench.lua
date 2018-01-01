function addTanks(numTanks)
   local terrain = app:getTerrain()
   local rot = helo.Vector3(0, 190, 0)
   for n = 1,numTanks do
      local x = 1780.96 + n * 20
      local z = 1635.3 + n * 20
      local y = terrain:getHeight(x, z)
      local pos = helo.Vector3(1780.96, 2.0, 1635.3)
      local tank = conf:loadVehicle("Abrams", "tsx-abrams." .. n,
				    helo.Vector3(x, y + 2.0, z),
				    rot)
      local po = tank:toPhysicsObject()
      po:finishPhysicsConfiguration(phys)

      local c = helo.SimpleCarAutoController(tank)
      tank:toControllable():setController(c)
      c:setSpeed(10.0)
   end
end


print("Setting up TSX benchmark...")
addTanks(20)
print("Done")
