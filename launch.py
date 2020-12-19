import krpc
import time
import math


def error(actual, expected):
    return abs(actual - expected) / expected


turn_start_altitude = 2500
turn_end_altitude = 50000
target_orbit = 150000

conn = krpc.connect(name="Launch Into Orbit")
vessel = conn.space_center.active_vessel

ut = conn.add_stream(getattr, conn.space_center, "ut")
altitude = conn.add_stream(getattr, vessel.flight(), "mean_altitude")
apoapsis = conn.add_stream(getattr, vessel.orbit, "apoapsis_altitude")
latitude = conn.add_stream(getattr, vessel.flight(), "latitude")
longitude = conn.add_stream(getattr, vessel.flight(), "longitude")
throttle = conn.add_stream(getattr, vessel.control, "throttle")

stage_one_fuel = conn.add_stream(
    vessel.parts.with_title("Ghidorah K1-180 Tank")[0].resources.amount, "LiquidFuel"
)

stage_one_initial_fuel = stage_one_fuel()

vessel.control.sas = False
vessel.control.rcs = False
vessel.control.throttle = 1.0

runmode = 0
first_stage_seperated = False
first_stage_sep_time = -1
runmode_names = [
    "Countdown",
    "Initial Ascent",
    "Gravity Turn",
    "Stage Seperation",
    "Burn till target apoapsis",
    "Coast to Circularization",
    "Circularization",
]

T0 = ut() + 10

print("\n")

while runmode != -1:
    if runmode == 0:
        if ut() >= T0:
            vessel.control.activate_next_stage()
            vessel.auto_pilot.engage()
            vessel.auto_pilot.target_pitch_and_heading(90, 90)
            runmode = 1
    if runmode == 1:
        if altitude() > turn_start_altitude:
            runmode = 2
    if runmode == 2:
        frac = (altitude() - turn_start_altitude) / (
            turn_end_altitude - turn_start_altitude
        )
        turn_angle = frac * 90
        vessel.auto_pilot.target_pitch_and_heading(90 - turn_angle, 90)

        if altitude() >= turn_end_altitude:
            runmode = 4

        if not first_stage_seperated and (
            stage_one_fuel() / stage_one_initial_fuel <= 0.25
        ):
            vessel.control.throttle = 0.0
            vessel.control.activate_next_stage()
            vessel.control.rcs = True
            vessel.control.throttle = 1.0
            runmode = 3
            first_stage_sep_time = ut()
    if runmode == 3:
        if ut() >= first_stage_sep_time + 3:
            vessel.control.activate_next_stage()
            first_stage_seperated = True
            runmode = 2
    if runmode == 4:
        ideal_throttle = max(min((target_orbit - apoapsis()) / 10, 100), 40)
        if abs(throttle() - ideal_throttle) > 0.5:
            vessel.control.throttle = ideal_throttle
        if apoapsis() >= target_orbit:
            vessel.control.throttle = 0
            runmode = 5

    print(f"Run mode: {runmode_names[runmode]}\t-\tT: {round(ut() - T0, 3)}", end="\r")
