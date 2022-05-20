#!/usr/bin/env python2.7
# Author: Hyungsub Kim
# Email: kim2956@purdue.edu
# Reference: https://www.ardusub.com/developers/pymavlink.html
# How to execute it?
### (1) Open a cmd
### (2) Run a SITL simulator (e.g., in the case of ArduPilot, ./Tools/autotest/sim_vehicle.py -v ArduCopter --console -w)
### (3) Open another cmd
### (4) python2 patch_type_analyzer.py

import os
import time
import math
import threading
from pymavlink import mavutil, mavwp
from pymavlink import mavextra
from pymavlink import mavexpression

# ------------------------------------------------------------------------------------
# Global variables
goal_throttle = 0
heartbeat_cnt = 0
current_flight_mode = ""
finished_send_triggering_inputs = 0
home_altitude = 0

# Global variables related to formulas
arming_inputs = 0 # Does bug triggering inputs contain the arming command? (i.e., user_command,400,1,0,0,0,0,0,0) (1: yes, 0: no)
violation_during_whole_stages = 0 # Does bug occur during whole flight stages? (1: yes, 0: no)

bug_triggering_inputs = []
bug_triggering_inputs_values = []
bug_triggering_inputs_types = []

target_param = ""
target_param_ready = 0
target_param_value = 0

# states - 0: turn off, 1: turn on
drone_status = 0
armed_state = 0
Parachute_on = 0
pre_arm_checks = 0
GPS_status = 1
Accel_status = 1
Gyro_status = 1
Baro_status = 1
current_yaw = 0
current_altitude = 0
previous_altitude = 0
current_heading = 0
flight_mode_history = []
flight_mode_history_alt = []

current_rc_3 = 0
current_roll = 0.0
current_pitch = 0.0
system_time = 0
mission_cnt = 0
# ------------------------------------------------------------------------------------

# ------------------------------------------------------------------------------------
# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
def set_rc_channel_pwm(id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if id < 1:
        print("Channel does not exist.")
        return

    # We only have 8 channels
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    if id < 9:
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[id - 1] = pwm

        # global master
        master.mav.rc_channels_override_send(
            master.target_system,  # target_system
            master.target_component,  # target_component
            *rc_channel_values)  # RC channel list, in microseconds.
# ------------------------------------------------------------------------------------

# ------------------------------------------------------------------------------------
def throttle_th():
    global goal_throttle

    while True:
        set_rc_channel_pwm(3, goal_throttle)
        time.sleep(0.2)
# ------------------------------------------------------------------------------------

# ------------------------------------------------------------------------------------
def read_loop():
    while True:
        # grab a mavlink message
        msg = master.recv_match(blocking=True)

        # handle the message based on its type
        msg_type = msg.get_type()
        if msg_type == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                sys.stdout.write(msg.data)
                sys.stdout.flush()

        elif msg_type == "RC_CHANNELS":
            handle_rc_raw(msg)

        elif msg_type == "VFR_HUD":
            handle_hud(msg)

        elif msg_type == "ATTITUDE":
            handle_attitude(msg)

        elif msg_type == "SYSTEM_TIME":
            handle_time(msg)

        elif msg_type == "MISSION_COUNT":
            handle_mission(msg)

        elif msg_type == "HEARTBEAT":
            handle_heartbeat(msg)

        elif msg_type == "STATUSTEXT":
            handle_status(msg)

        elif msg_type == "PARAM_VALUE":
            handle_param(msg)


# ------------------------------------------------------------------------------------

# --------------------- (Start) READ Robotic Vehicle's states ------------------------
def handle_heartbeat(msg):
    global heartbeat_cnt
    heartbeat_cnt += 1

    global current_flight_mode
    global current_altitude

    if current_flight_mode != mavutil.mode_string_v10(msg).strip():
        current_flight_mode = mavutil.mode_string_v10(msg)
        flight_mode_history.append(current_flight_mode)
        flight_mode_history_alt.append(current_altitude)

    global drone_status
    drone_status = msg.system_status
    #print("[DEBUG] Drone status: %d, mavlink version: %d" % (drone_status, msg.mavlink_version))

    global armed_state
    is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    is_enabled = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED

    if is_armed == 128:
        armed_state = 1 # 1 denotes the RV is armed.
    elif is_armed == 0:
        armed_state = 0 # 0 denotes the RV is disarmed.

    #print("[DEBUG] Mode: %s, Armed: %d" % (current_flight_mode, is_armed))
# ------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------
def handle_rc_raw(msg):
    global current_rc_3
    current_rc_3 = msg.chan3_raw

    channels = (msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw)


# ------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------
def handle_status(msg):
    global Parachute_on
    global GPS_status
    global Accel_status
    global Gyro_status
    global pre_arm_checks
    global Baro_status

    status_data = (msg.severity, msg.text)
    print("[status_text] %s" % msg.text)

    # Detecting a depolyed parachute
    if "Parachute: Released" in msg.text:
        Parachute_on = 1

    # Detecting an error on GPS
    elif "NavEKF" in msg.text and "lane switch" in msg.text:
        GPS_status = 0

    # Detecting an error on gyro sensor
    elif "Vibration compensation ON" in msg.text:
        Gyro_status = 0

    # Detecting an error on accelerometer sensor
    elif "EKF primary changed" in msg.text:
        Accel_status = 0

    # Detecting error messages related to a barometer sensor
    elif "PreArm: Waiting for Nav Checks" in msg.text:
        Baro_status = 0

    # Detecting a PreArm check error
    elif "PreArm" in msg.text:
        pre_arm_checks = 1
# ------------------------------------------------------------------------------------

# ------------------------------------------------------------------------------------
def handle_param(msg):
    global target_param
    global target_param_ready
    global target_param_value

    message = msg.to_dict()

    #print('[handle_param] target_param: %s vs. name: %s\tvalue: %d' % (target_param, message['param_id'], message['param_value']))

    if message['param_id'].decode("utf-8").strip() == target_param.strip():
        target_param_ready = 1
        target_param_value = message['param_value']
    else:
        target_param_ready = 0

# ------------------------------------------------------------------------------------

# ------------------------------------------------------------------------------------
def handle_hud(msg):
    global current_yaw

    hud_data = (msg.airspeed, msg.groundspeed, msg.heading,
                msg.throttle, msg.alt, msg.climb)
    # print "Aspd\tGspd\tHead\tThro\tAlt\tClimb"
    # print "%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f" % hud_data

    # print("Alt: %f" %msg.alt)

    global current_altitude
    global previous_altitude
    global current_heading

    previous_altitude = current_altitude
    current_altitude = msg.alt

    current_heading = (msg.heading - 359)


# ------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------
def handle_attitude(msg):
    global current_roll
    global current_pitch

    attitude_data = (msg.roll, msg.pitch, msg.yaw, msg.rollspeed,
                     msg.pitchspeed, msg.yawspeed)
    # print "Roll\tPit\tYaw\tRSpd\tPSpd\tYSpd"
    # print "%0.6f\t%0.6f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t" % attitude_data

    current_roll = (msg.roll * 180) / math.pi
    current_pitch = (msg.pitch * 180) / math.pi


# ------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------
def handle_time(msg):
    global system_time

    # time_data = (msg.time_unix_usec, msg.time_boot_ms)
    system_time = msg.time_unix_usec


# print "[system_time] UNIX time:%d, Boot time:%d" % time_data

# ------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------
def handle_mission(msg):
    global mission_cnt

    mission_cnt = msg.count
    print("[MISSION_COUNT]%d" % mission_cnt)


# ------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------
def trigger_logic_bug():
    global arming_inputs
    global bug_triggering_inputs
    global bug_triggering_inputs_values
    global bug_triggering_inputs_types

    dirname = os.path.dirname(__file__)
    filename = "../" + "/1b_bug_triggering_inputs.txt"
    print("[DEBUG] 1b_bug_triggering_inputs Directory: %s" %dirname)
    filename = os.path.join(dirname, filename)

    predicate_name = []
    predicate_expected_value = []

    with open(filename) as file_read:
        line = file_read.readline()
        while line:
            line = file_read.readline()

            if "###" in line:
                continue

            elif "predicate" in line:
                input = line.split(",")
                predicate_name.append(input[1])
                predicate_expected_value.append(input[2])

            else:
                input = line.split(",")
                print("[DEBUG] line:%s" %line)
                # 1) When the input is a configuration parameter/environmental factors
                if (input[0] == "parameter") or (input[0] == "env"):
                    # 1-1) Set parameter value
                    print("[DEBUG] %s, %f" %(input[1], float(input[2])))
                    master.mav.param_set_send(master.target_system, master.target_component,
                                              input[1],
                                              float(input[2]),
                                              mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

                    if input[0] == "parameter":
                        bug_triggering_inputs_types.append("parameter")
                    elif input[0] == "env":
                        bug_triggering_inputs_types.append("env")

                    bug_triggering_inputs.append(input[1])
                    bug_triggering_inputs_values.append(input[2])
                    time.sleep(3)

                # 2) When the input is a user command
                if input[0] == "user_command":
                    # Case 2-1) When the user command is changing the RV's flight mode
                    if input[1].strip() == "mode":
                        target_mode = input[2].strip()
                        # Check if mode is available
                        if target_mode not in master.mode_mapping():
                            print('Unknown mode : {}'.format(target_mode))
                            print('Try:', list(master.mode_mapping().keys()))
                            exit(1)

                        # Get mode ID
                        mode_id = master.mode_mapping()[target_mode]

                        master.mav.set_mode_send(
                            master.target_system,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            mode_id)

                    else:
                        master.mav.command_long_send(
                            master.target_system,  # target_system
                            master.target_component,  # target_component
                            int(input[1]),  # command
                            0,  # confirmation
                            float(input[2]),  # param1
                            float(input[3]),  # param2
                            float(input[4]),  # param3
                            float(input[5]),  # param4
                            float(input[6]),  # param5
                            float(input[7]),  # param6
                            float(input[8]))  # param7

                        time.sleep(1)
                        if (int(input[1]) == 400) and (int(input[2]) == 1): # When the bug triggering inputs contain 'arming' command
                            arming_inputs = 1
                            global goal_throttle
                            goal_throttle = 1500

                        if int(input[1]) == 22: # when the user command was 'take-off' command, we need to wait for longer time.
                            time.sleep(5)

                    bug_triggering_inputs.append(input[1])
                    time.sleep(3)

    global finished_send_triggering_inputs
    finished_send_triggering_inputs = 1
# ------------------------------------------------------------------------------------

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
# Wait a heartbeat before sending commands
master.wait_heartbeat()
time.sleep(5)

# request data to be sent at the given rate
for i in range(0, 3):
    master.mav.request_data_stream_send(master.target_system, master.target_component,
                                        mavutil.mavlink.MAV_DATA_STREAM_ALL, 6, 1)

# https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
"""
# Arm
# master.arducopter_arm() or:
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

print("[DEBUG]%s arm/disarm user command" % mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)

# wait until arming confirmed (can manually check with master.motors_armed())
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')
"""

"""
# Disarm
# master.arducopter_disarm() or:
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

# wait until disarming confirmed
master.motors_disarmed_wait()
"""

goal_throttle = 1000
t1 = threading.Thread(target=throttle_th, args=())
t1.daemon = True
t1.start()

time.sleep(3)

t2 = threading.Thread(target=read_loop, args=())
t2.daemon = True
t2.start()

time.sleep(3)

home_altitude = current_altitude

t3 = threading.Thread(target=trigger_logic_bug, args=())
t3.daemon = True
t3.start()

# Execute a function that trigger the buggy behavior

# Main loop
while True:

    time.sleep(3)

    # 3) Check whether the RV's state violate the predicates or not

    if finished_send_triggering_inputs == 1:
        dirname = os.path.dirname(__file__)
        filename = "../" + "/3b_term_classification_table.txt"
        print("[DEBUG] Directory: %s" % dirname)
        filename = os.path.join(dirname, filename)

        precondition_types = []
        precondition_terms = [] # Term-1
        precondition_operator = [] # Operator
        precondition_values = [] # Term-2
        precondition_violated_type = []

        postcondition_types = []
        postcondition_terms = [] # Term-1
        postcondition_operator = [] # Operator
        postcondition_values = [] # Term-2
        postcondition_violated_type = []

        post_condition = 0
        with open(filename) as file_read:

            line = file_read.readline()
            #print("[DEBUG] line:%s" %line)

            while line:
                line = file_read.readline()
                if "###" in line:
                    continue

                print("[DEBUG] line:%s" % line)
                input = line.split(",")
                #print("[DEBUG] len(input):%s" % len(input))

                if len(input) <= 1:
                    break

                if "~" in input[1].strip():
                    post_condition = 1

                # 1) When the term is the RV's physical state
                # input[2] == term-1, input[3] == operator, input[4] == term-2,
                # input[6] == term-1's type, input[6] == term-2's type,
                #
                if input[6].strip() == "physical_state":
                    print("[DEBUG] term is physical_state")

                    # 1-1) Term-1
                    if input[2].strip() == "armed":
                        if post_condition == 0:
                            precondition_types.append("physical_state")
                            precondition_terms.append("armed") # Term-1
                        else:
                            postcondition_types.append("physical_state")
                            postcondition_terms.append("armed") # Term-1

                    # 1-2) Operator
                    if post_condition == 0:
                        precondition_operator.append(input[3].strip()) # Operator
                    else:
                        postcondition_operator.append(input[3].strip()) # Operator

                    # 1-3) Term-2
                    if input[4].strip() == "false":
                        if post_condition == 0:
                            precondition_values.append(0) # Term-2
                        else:
                            postcondition_values.append(0) # Term-2

                    if input[4].strip() == "true":
                        if post_condition == 0:
                            precondition_values.append(1) # Term-2
                        else:
                            postcondition_values.append(1) # Term-2

                    # 1-4) Other cases (Store Term-1 and Term-2)
                    else:
                        if post_condition == 0:
                            precondition_types.append("physical_state")
                            precondition_terms.append(input[2].strip()) # Term-1
                            if input[4].strip() != "":
                                precondition_values.append(input[4].strip()) # Term-2
                        else:
                            postcondition_types.append("physical_state")
                            postcondition_terms.append(input[2].strip()) # Term-2
                            if input[4].strip() != "":
                                postcondition_values.append(input[4].strip()) # Term-2



                # 2) When the term is a configuration parameter
                # input[2] == term-1, input[3] == operator, input[4] == term-2,
                # input[6] == term-1's type, input[6] == term-2's type,
                #
                elif input[6].strip() == "configuration_parameter":
                    print("[DEBUG] term is configuration_parameter")

                    if post_condition == 0:
                        precondition_types.append("configuration_parameter")
                        precondition_terms.append(input[2].strip()) # Term-1
                        precondition_operator.append(input[3].strip()) # Operator
                        precondition_values.append(input[4].strip()) # Term-2
                    else:
                        postcondition_types.append("configuration_parameter")
                        postcondition_terms.append(input[2].strip()) # Term-1
                        postcondition_operator.append(input[3].strip()) # Operator
                        postcondition_values.append(input[4].strip()) # Term-2

                # 3) When the term is a function/variable
                # input[2] == term-1, input[3] == operator, input[4] == term-2,
                # input[6] == term-1's type, input[6] == term-2's type,
                #
                elif (input[6].strip() == "function") or (input[6].strip() == "variable"):
                    print("[DEBUG] term is a function/variable")

                    # 1-1) Term type
                    if post_condition == 0:
                        precondition_types.append("function/variable")
                    else:
                        postcondition_types.append("function/variable")

                    # 1-2) Operator
                    if post_condition == 0:
                        precondition_operator.append(input[3].strip()) # Operator
                    else:
                        postcondition_operator.append(input[3].strip()) # Operator

                    if input[2].strip() == "pre_arm_checks":
                        if input[4].strip() == "false":
                            if post_condition == 0:
                                precondition_terms.append("PreArm")
                                precondition_values.append(0)
                            else:
                                postcondition_terms.append("PreArm")
                                postcondition_values.append(0)

                        if input[4].strip() == "true" or input[4].strip() == "error":
                            if post_condition == 0:
                                precondition_terms.append("PreArm")
                                precondition_values.append(1)
                            else:
                                postcondition_terms.append("PreArm")
                                postcondition_values.append(1)

                    # Other cases (Store Term-1 and Term-2)
                    else:
                        if post_condition == 0:
                            precondition_terms.append(input[2].strip()) # Term-1
                            precondition_operator.append(input[3].strip()) # Operator
                            if input[4].strip() != "":
                                precondition_values.append(input[4].strip()) # Term-2
                        else:
                            postcondition_terms.append(input[2].strip()) # Term-1
                            postcondition_operator.append(input[3].strip())  # Operator
                            if input[4].strip() != "":
                                postcondition_values.append(input[4].strip()) # Term-2


        # To-do: I need to handle ">, <, <=, >=, !=" on formulas
        # Match formula with the RV's states
        # ---------- (Start) Check whether pre-conditions and post-conditions are satisfied or not ----------

        # Does the RV's states violate the pre-conditions?
        ### 1: Violation, 0: No violation
        pre_condition_violation = 0

        # Does the RV's states violate the post-conditions?
        ### 1: Violation, 0: No violation
        post_condition_violation = 0

        print("------------ (Start) [Match formula with the RV's states] ------------")

        print("[DEBUG] # of terms in the pre-condition: %d" % len(precondition_types))
        print("[DEBUG] # of terms in the post-condition: %d" % len(postcondition_types))

        """
            Notice to Users: You need to add your predicates in below to detect the buggy behavior created by a logic bug.
        """

        buggy_behavior = 0
        for itr in range(len(precondition_types)):
            # Case-1: precondition type is 'physical_state' or 'function/variable'
            if (precondition_types[itr] == "physical_state") or (precondition_types[itr] == "function/variable"):

                # If the term is 'PreArm'
                if precondition_terms[itr] == "PreArm":
                    # When the operator is '='
                    if precondition_operator[itr] == "=":
                        if int(precondition_values[itr]) != pre_arm_checks:
                            print("[DEBUG] The formula's preconditions is not satisfied because of function.")
                            buggy_behavior = 1

                    # When the operator is '!='
                    elif precondition_operator[itr] == "!=":
                        if int(precondition_values[itr]) == pre_arm_checks:
                            print("[DEBUG] The formula's preconditions is not satisfied because of function.")
                            buggy_behavior = 1
                    else:
                        print("[ERROR] Incorrect operator: %s" % precondition_operator[itr])

                # If the term is 'armed'
                elif precondition_terms[itr] == "armed":
                    # When the operator is '='
                    if precondition_operator[itr] == "=":
                        if int(precondition_values[itr]) != armed_state:
                            print("[DEBUG] expected value:%d, armed_state:%d" %(precondition_values[itr], armed_state))
                            print("[DEBUG] The formula's preconditions is not satisfied because of physical_state.")
                            buggy_behavior = 1

                    # When the operator is '!='
                    elif precondition_operator[itr] == "!=":
                        if int(precondition_values[itr]) == armed_state:
                            print("[DEBUG] %d %s %d" %(precondition_values[itr], precondition_operator[itr], armed_state))
                            print("[DEBUG] The formula's preconditions is not satisfied because of physical_state.")
                            buggy_behavior = 1

                # If the term is 'altitude'
                elif precondition_terms[itr] == "altitude":

                    # When the operator is '='
                    if precondition_operator[itr] == "=":
                        if (previous_altitude - home_altitude) != float(precondition_values[itr]):
                            buggy_behavior = 1

                    # When the operator is '!='
                    if precondition_operator[itr] == "!=":
                        if (previous_altitude - home_altitude) == float(precondition_values[itr]):
                            buggy_behavior = 1

                    # When the operator is '>='
                    if (precondition_operator[itr] == ">=") or (precondition_operator[itr] == ">"):
                        if (previous_altitude - home_altitude) < float(precondition_values[itr]):
                            buggy_behavior = 1

                    # When the operator is '<='
                    if (precondition_operator[itr] == "<=") or (precondition_operator[itr] == "<"):
                        if (previous_altitude - home_altitude) > float(precondition_values[itr]):
                            buggy_behavior = 1

                    if buggy_behavior == 1:
                        print("[DEBUG] %f %s %f" % (float(precondition_values[itr]), precondition_operator[itr], float(previous_altitude)))
                        precondition_violated_type.append("physical_state/function/variable")

            elif precondition_types[itr] == "configuration_parameter":

                # Request all parameters
                master.mav.param_request_list_send(
                    master.target_system, master.target_component
                )

                target_param = precondition_terms[itr]
                count = 0
                while (target_param_ready == 0) and (count < 5):
                    time.sleep(1)
                    count += 1

                print("[DEBUG] param:%s, value: %s" % (precondition_terms[itr], target_param_value))

                time.sleep(3)

                if float(precondition_values[itr]) != target_param_value:
                    print("[DEBUG] expected value:%s, target_param_value:%s" % (precondition_values[itr], target_param_value))
                    print("[DEBUG] The formula's preconditions is not satisfied because of parameters.")
                    buggy_behavior = 1
                    precondition_violated_type.append("configuration_parameter")


        if buggy_behavior == 0:
            print("[ANALYSIS_RESULT] The formula's preconditions is satisfied.")
        else:
            pre_condition_violation = 1 # Violation occurs from pre-conditions part
            print("[ANALYSIS_RESULT] The formula's preconditions is not satisfied.")

        itr = 0
        buggy_behavior = 0
        print("[DEBUG] len(type):%d, len(name):%d, len(value):%d" % (
        len(postcondition_types), len(postcondition_terms), len(postcondition_values)))

        for itr in range(len(postcondition_types)):

            print("[DEBUG] type:%s, name:%s, value:%s"%(postcondition_types[itr], postcondition_terms[itr], postcondition_values[itr]))

            if (postcondition_types[itr] == "physical_state") or (postcondition_types[itr] == "function/variable"):

                if postcondition_terms[itr] == "PreArm":
                    if int(postcondition_values[itr]) != pre_arm_checks:
                        print("[DEBUG] The formula's postconditions is not satisfied because of function.")
                        buggy_behavior = 1

                elif postcondition_terms[itr] == "armed":
                    if int(postcondition_values[itr]) != armed_state:
                        print("[DEBUG] expected value:%d, armed_state:%d" % (postcondition_values[itr], armed_state))
                        print("[DEBUG] The formula's postconditions is not satisfied because of physical_state.")
                        buggy_behavior = 1

                elif postcondition_terms[itr] == "mode":

                    for i in range(len(flight_mode_history)):
                        # When the operator is '='
                        if postcondition_operator[itr] == "=":
                            if (flight_mode_history[i] != postcondition_values[itr]) and (pre_condition_violation == 0):
                                print("[DEBUG] %s %s %s" % (
                                    flight_mode_history[i], postcondition_operator[itr], postcondition_values[itr]))
                                buggy_behavior = 1

                        # When the operator is '!='
                        if postcondition_operator[itr] == "!=":
                            if (flight_mode_history[i] == postcondition_values[itr]) and (pre_condition_violation == 0):
                                print("[DEBUG] %s %s %s" % (
                                    flight_mode_history[i], postcondition_operator[itr], postcondition_values[itr]))
                                buggy_behavior = 1

                if buggy_behavior == 1:
                    postcondition_violated_type.append("physical_state/function/variable")

            elif postcondition_types[itr] == "configuration_parameter":

                # Request all parameters
                master.mav.param_request_list_send(
                    master.target_system, master.target_component
                )

                target_param = postcondition_terms[itr]
                count = 0
                while (target_param_ready == 0) and (count < 5):
                    time.sleep(1)
                    count += 1

                print("[DEBUG] param:%s, value: %s" % (postcondition_terms[itr], target_param_value))

                time.sleep(3)

                if float(postcondition_values[itr]) != target_param_value:
                    print("[DEBUG] expected value:%s, target_param_value:%s" % (
                    postcondition_values[itr], target_param_value))
                    print("[DEBUG] The formula's postconditions is not satisfied because of parameters.")
                    buggy_behavior = 1
                    postcondition_violated_type.append("configuration_parameter")

        if buggy_behavior == 0:
            print("[ANALYSIS_RESULT] The formula's postconditions is satisfied.")
        else:
            post_condition_violation = 1  # Violation occurs from post-conditions part
            print("[ANALYSIS_RESULT] The formula's postconditions is not satisfied.")

        print("------------ (End) [Match formula with the RV's states] ------------")
        # ---------- (End) Check whether pre-conditions and post-conditions are satisfied or not ----------

        print("------------ (Start) [What is the recommended patch type?] ------------")

        patch_type = "" # This variable will contain 'patch type'.
        reason_details = "" # This variable will contain a sentence to describe why we decide the 'patch type'.

        # ----------------------- [Patch Type Inference] -----------------------
        # ----------------------- [Patch Type Inference] -----------------------
        print("[DEBUG] pre_condition_violation: %d" % pre_condition_violation)
        # ----------------------- (Start) 1. Patch type is "ADD" -----------------------
        ### Satisfy pre-conditions but violate post-conditions
        if (pre_condition_violation == 0) and (post_condition_violation == 1):
            ### Case ADD-1: When the buggy behavior occurs before take-off/arming stages (i.e., if 'arming_inputs' is 0)
            if arming_inputs == 0:
                reason_details = "(1) Satisfy pre-conditions but violate post-conditions and (2) the buggy behavior occurs before take-off/arming stages"
                patch_type = "ADD"
                print("[DEBUG] patch candidate: %s" % patch_type)

            ### Case ADD-2: When the buggy behavior occurs during the whole flight stages
            if violation_during_whole_stages == 0:
                reason_details = "(1) Satisfy pre-conditions but violate post-conditions and (2) the buggy behavior occurs during the whole flight stages"
                patch_type = "ADD"
                print("[DEBUG] patch candidate: %s" % patch_type)
        # ----------------------- (End) 1. Patch type is "ADD" -----------------------


        # ----------------------- (Start) 2. Patch type is "UPDATE" -----------------------
        ### Violate pre-conditions and satisfy post-conditions
        if (pre_condition_violation == 1) and (post_condition_violation == 0) and (len(postcondition_terms) >= 1):
            reason_details = "Violate pre-conditions and satisfy post-conditions"
            patch_type = "UPDATE"
            print("[DEBUG] patch candidate: %s" % patch_type)
        # ----------------------- (END) 2. Patch type is "UPDATE" -----------------------

        # ----------------------- (Start) 3. Patch type is "DISABLE" -----------------------
        if (pre_condition_violation == 1) or (post_condition_violation == 1):
            disable = 0

            for itr in range(len(precondition_types)):
                if "disabled" in precondition_terms[itr]:
                    disable = 1

            for itr in range(len(postcondition_types)):
                if "disabled" in postcondition_terms[itr]:
                    disable = 1

            if disable == 1:
                reason_details = "Formula explicitly mentions disable a behavior"
                patch_type = "DISABLE"
                print("[DEBUG] patch candidate: %s" % patch_type)
        # ----------------------- (END) 3. Patch type is "DISABLE" -----------------------

        # ----------------------- (Start) 4. Patch type is "REUSE" -----------------------
        if (pre_condition_violation == 0) and (post_condition_violation == 1):
            if (arming_inputs == 1) and (violation_during_whole_stages == 0):
                # If the violation occurs from a configuration parameter, we consider this case as 'REUSE'

                reuse = 0
                for i in range(len(postcondition_violated_type)):
                    if postcondition_violated_type[i] == "configuration_parameter":
                        reuse += 1

                if reuse != 0:
                    reason_details = "(1) Satisfy pre-conditions but violate post-conditions and (2) the buggy behavior occurs after take-off/arming stages"
                    patch_type = "REUSE"
                    print("[DEBUG] patch candidate: %s" % patch_type)
        # ----------------------- (END) 4. Patch type is "REUSE" -----------------------

        # ----------------------- (Start) 5. Patch type is "CHECK" -----------------------
        ### A parameter value outside the valid range leads to the policy violation
        if (pre_condition_violation == 1) and (len(postcondition_types) == 0):
            find_root_case = 0
            for itr in range(len(bug_triggering_inputs_types)):

                if bug_triggering_inputs_types[itr] == "parameter":
                    # Case 1: When the RV software is ArduPilot
                    for iteration in range(6):
                        dirname = os.path.dirname(__file__)

                        if iteration == 0:
                            filename = "../" + "/4_term_analyzer/ArduPilot/output_copter.csv"
                        elif iteration == 1:
                            filename = "../" + "/4_term_analyzer/ArduPilot/output_plane.csv"
                        elif iteration == 2:
                            filename = "../" + "/4_term_analyzer/ArduPilot/output_rover.csv"
                        elif iteration == 3:
                            filename = "../" + "/4_term_analyzer/ArduPilot/output_submarine.csv"
                        elif iteration == 4:
                            filename = "../" + "/4_term_analyzer/ArduPilot/output_antenna_tracker.csv"
                        elif iteration == 5:
                            filename = "../" + "/4_term_analyzer/PX4/output_copter.csv"


                        filename = os.path.join(dirname, filename)

                        with open(filename) as file_read:
                            line = file_read.readline()
                            while (line) and (find_root_case == 0):
                                #print("[DEBUG] line from csv: %s" % line)
                                row = line.split(",")

                                # Extract the range
                                if (row[0].strip() == bug_triggering_inputs[itr]) and (row[2].strip() is not ""):
                                    range = row[2].split(" ")
                                    min = float(range[0])
                                    max = float(range[1])

                                    if (float(bug_triggering_inputs_values[itr]) < min) or (float(bug_triggering_inputs_values[itr]) > max):
                                        print("[ROOT_CAUSE] parameter: %s, Min: %f, Max: %f, assigned value: %f" % (
                                        bug_triggering_inputs[itr], min, max, float(bug_triggering_inputs_values[itr])))

                                        reason_details = "parameter:" + bug_triggering_inputs[itr] + ", Min:" + str(
                                            min) + ", Max:" + str(max) + ", assigned value:" + \
                                                         bug_triggering_inputs_values[itr]
                                        patch_type = "CHECK"
                                        find_root_case = 1

                                line = file_read.readline()

                    # We already find the root case. Let's finish this algorithm.
                    if find_root_case == 1:
                        break

                    # Paparazzi
                    dirname = os.path.dirname(__file__)
                    filename = "../" + "/4_term_analyzer/Paparazzi/ctl_basic.xml.csv"
                    filename = os.path.join(dirname, filename)

                    with open(filename) as file_read:
                        line = file_read.readline()
                        while (line) and (find_root_case == 0):

                            row = line.split(",")

                            # Extract the range
                            if (row[0].strip() == bug_triggering_inputs[itr]) and (row[2].strip() is not ""):
                                min = float(row[2])
                                max = float(row[3])

                                if (float(bug_triggering_inputs_values[itr]) < min) or (float(bug_triggering_inputs_values[itr]) > max):
                                    print("[ROOT_CAUSE] parameter: %s, Min: %f, Max: %f, assigned value: %f" % (
                                    bug_triggering_inputs[itr], min, max, float(bug_triggering_inputs_values[itr])))

                                    reason_details = "parameter:" + bug_triggering_inputs[itr] + ", Min:" + str(
                                        min) + ", Max:" + str(max) + ", assigned value:" + \
                                                     bug_triggering_inputs_values[itr]
                                    patch_type = "CHECK"
                                    find_root_case = 1

                            line = file_read.readline()

                    # We already find the root case. Let's finish this algorithm.
                    if find_root_case == 1:
                        break

        # ----------------------- (END) 5. Patch type is "CHECK" -----------------------

        ### End this program after finishing to infer the patch type
        if patch_type is not "":
            print("[FINAL_ANALYSIS_RESULT] What is the recommended patch type? %s patch type." % patch_type)
            print("[REASON] %s" % reason_details)

            dirname = os.path.dirname(__file__)
            filename = "../" + "6_patch_type.txt"
            filename = os.path.join(dirname, filename)
            f = open(filename, "w")
            f.write("### '###'denotes a comment. ###\n")
            f.write("### Patch type inference result\n")
            f.write("### Reason: %s\n" % reason_details)
            f.write("%s" % patch_type)
            f.close()
            break

        print("------------ (End) [What is the recommended patch type?] ------------")

        if (finished_send_triggering_inputs == 1) and (patch_type == ""):
            print("Sorry, we fail to infer the patch type")

    else:
        print("Ongoing...")