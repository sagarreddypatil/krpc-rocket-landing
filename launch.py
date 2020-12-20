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
time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, "time_to_apoapsis")

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
    "Coast to Space",
    "Coast to Circularization",
    "Circularization",
    "Circularization Fine-Tuning",
]

T0 = ut() + 10
circularization_node = None
node_dv = None
burn_time = -1

print("\n")

while runmode != -1:
    print(
        f"Run mode: {runmode_names[runmode]}\t-\tT: {round(ut() - T0, 1)}                  ",
        end="\r",
    )

    if runmode == 0:
        if ut() >= T0:
            vessel.control.activate_next_stage()
            vessel.auto_pilot.engage()
            vessel.auto_pilot.target_pitch_and_heading(90, 90)
            runmode = 1
            continue
    if runmode == 1:
        if altitude() > turn_start_altitude:
            runmode = 2
            continue
    if runmode == 2:
        frac = (altitude() - turn_start_altitude) / (
            turn_end_altitude - turn_start_altitude
        )
        turn_angle = frac * 90
        vessel.auto_pilot.target_pitch_and_heading(90 - turn_angle, 90)

        if altitude() >= turn_end_altitude:
            runmode = 4
            continue

        if not first_stage_seperated and (
            stage_one_fuel() / stage_one_initial_fuel <= 0.25
        ):
            vessel.control.throttle = 0.0
            vessel.control.activate_next_stage()
            vessel.control.rcs = True
            vessel.control.throttle = 1.0
            runmode = 3
            first_stage_sep_time = ut()
            continue
    if runmode == 3:
        if ut() >= first_stage_sep_time + 3:
            vessel.control.activate_next_stage()
            first_stage_seperated = True
            runmode = 2
            continue
    if runmode == 4:
        ideal_throttle = max(min((target_orbit - apoapsis()) / 1000, 1), 0.4)
        if abs(throttle() - ideal_throttle) > 0.5:
            vessel.control.throttle = ideal_throttle
        if apoapsis() >= target_orbit:
            vessel.control.throttle = 0
            runmode = 5
            continue
    if runmode == 5:
        if apoapsis() < target_orbit:
            vessel.control.throttle = 0.4
        if (apoapsis() >= target_orbit) and (throttle() > 0):
            vessel.control.throttle = 0
        if altitude() >= 70000:
            vessel.control.activate_next_stage()

            mu = vessel.orbit.body.gravitational_parameter
            r = vessel.orbit.apoapsis
            a1 = vessel.orbit.semi_major_axis
            a2 = r
            v1 = math.sqrt(mu * ((2.0 / r) - (1.0 / a1)))
            v2 = math.sqrt(mu * ((2.0 / r) - (1.0 / a2)))
            delta_v = v2 - v1
            circularization_node = vessel.control.add_node(
                ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v
            )
            node_dv = conn.add_stream(
                getattr, circularization_node, "remaining_delta_v"
            )

            F = vessel.available_thrust
            Isp = vessel.specific_impulse * 9.82
            m0 = vessel.mass
            m1 = m0 / math.exp(delta_v / Isp)
            flow_rate = F / Isp
            burn_time = (m0 - m1) / flow_rate

            vessel.auto_pilot.reference_frame = circularization_node.reference_frame
            vessel.auto_pilot.target_direction = (0, 1, 0)
            vessel.auto_pilot.wait()
            conn.space_center.warp_to(ut() + time_to_apoapsis() - (burn_time / 2) - 5)

            runmode = 6
            continue
    if runmode == 6:
        if time_to_apoapsis() - (burn_time / 2) < 0:
            vessel.control.throttle = 1.0
            runmode = 7
            continue
    if runmode == 7:
        if node_dv() < 100 and throttle() > 0.4:
            vessel.control.throttle = 0.4
            runmode = 8
            continue
    if runmode == 8:
        if node_dv() < 5:
            vessel.control.throttle = 0
            runmode = -1
            continue
