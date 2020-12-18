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
stage_2_resources = vessel.resources_in_decouple_stage(stage=2, cumulative=False)

vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 1.0

for i in range(3, 0, -1):
    print(f"{i}...")
    time.sleep(1)

vessel.control.activate_next_stage()
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(90, 90)

turn_angle = 0
while True:
    # Gravity turn
    if altitude() > turn_start_altitude and altitude() < turn_end_altitude:
        frac = (altitude() - turn_start_altitude) / (
            turn_end_altitude - turn_start_altitude
        )
        new_turn_angle = frac * 90
        if abs(new_turn_angle - turn_angle) > 0.5:
            turn_angle = new_turn_angle
            vessel.auto_pilot.target_pitch_and_heading(90 - turn_angle, 90)

    # Separate SRBs when finished
    if not srbs_separated:
        if srb_fuel() < 0.1:
            vessel.control.activate_next_stage()
            srbs_separated = True
            print("SRBs separated")

    # Decrease throttle when approaching target apoapsis
    if apoapsis() > target_altitude * 0.9:
        print("Approaching target apoapsis")
        break
