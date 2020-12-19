import krpc
import time
import math

turn_start_altitude = 250
turn_end_altitude = 45000
target_altitude = 150000

conn = krpc.connect(name="Launch Into Orbit")
vessel = conn.space_center.active_vessel

ut = conn.add_stream(getattr, conn.space_center, "ut")
altitude = conn.add_stream(getattr, vessel.flight(), "mean_altitude")
apoapsis = conn.add_stream(getattr, vessel.orbit, "apoapsis_altitude")

first_stage_tank = vessel.parts.with_title("Ghidorah K1-180 Tank")[0]
# second_stage_tank = vessel.parts.with_title("Ghidorah K2-81 Tank")[0]

stage_one_fuel = conn.add_stream(first_stage_tank.resources.amount, "LiquidFuel")
# stage_two_fuel = conn.add_stream(second_stage_tank.resources.amount, "LiquidFuel")

stage_one_initial_fuel = stage_one_fuel()

vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 1.0

for i in range(3, 0, -1):
    print(f"{i}...")
    time.sleep(1)

vessel.control.activate_next_stage()
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(90, 90)

second_stage_seperation = False

while True:
    # Gravity turn
    if altitude() > turn_start_altitude and altitude() < turn_end_altitude:
        frac = (altitude() - turn_start_altitude) / (
            turn_end_altitude - turn_start_altitude
        )
        turn_angle = frac * 90
        vessel.auto_pilot.target_pitch_and_heading(90 - turn_angle, 90)

    # Separate SRBs when finished
    if not second_stage_seperation:
        if stage_one_fuel() / stage_one_initial_fuel < 0.25:
            vessel.control.throttle = 0.0
            print("MECO")

            vessel.control.activate_next_stage()
            vessel.control.rcs = True
            vessel.control.throttle = 1.0

            vessel.control.activate_next_stage()
            second_stage_seperation = True

    # Decrease throttle when approaching target apoapsis
    if apoapsis() > target_altitude * 0.9:
        print("Approaching target apoapsis")
        break

vessel.control.throttle = 0.4

while apoapsis() < target_altitude:
    pass

print("Target apoapsis reached")
vessel.control.throttle = 0.0

while altitude() < 70500:
    pass

print("Reached Space")
vessel.control.activate_next_stage()

print("Planning circularization burn")
mu = vessel.orbit.body.gravitational_parameter
r = vessel.orbit.apoapsis
a1 = vessel.orbit.semi_major_axis
a2 = r
v1 = math.sqrt(mu * ((2.0 / r) - (1.0 / a1)))
v2 = math.sqrt(mu * ((2.0 / r) - (1.0 / a2)))
delta_v = v2 - v1
node = vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

# Calculate burn time (using rocket equation)
F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82
m0 = vessel.mass
m1 = m0 / math.exp(delta_v / Isp)
flow_rate = F / Isp
burn_time = (m0 - m1) / flow_rate

# Orientate ship
print("Orientating ship for circularization burn")
vessel.auto_pilot.reference_frame = node.reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)
vessel.auto_pilot.wait()

# Wait until burn
print("Waiting until circularization burn")
burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time / 2.0)
lead_time = 5
conn.space_center.warp_to(burn_ut - lead_time)

# Execute burn
print("Ready to execute burn")
time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, "time_to_apoapsis")
while time_to_apoapsis() - (burn_time / 2.0) > 0:
    pass
print("Executing burn")
vessel.control.throttle = 1.0
time.sleep(burn_time - 0.1)
print("Fine tuning")
vessel.control.throttle = 0.05
remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)
while remaining_burn()[1] > 0:
    pass
vessel.control.throttle = 0.0
node.remove()

print("Launch complete")

