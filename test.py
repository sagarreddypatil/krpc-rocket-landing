import krpc

conn = krpc.connect("Testing", address="192.168.1.178")
vessel = conn.space_center.active_vessel

first_stage_tank = vessel.parts.with_title("Ghidorah K1-180 Tank")[0]
second_stage_tank = vessel.parts.with_title("Ghidorah K2-81 Tank")[0]

print(first_stage_tank.resources.amount("LiquidFuel"))
print(second_stage_tank.resources.amount("LiquidFuel"))
