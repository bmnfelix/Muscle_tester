import os
import PySimpleGUI as sg
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
import serial
import keyboard
import nidaqmx
import csv
import sys
import math
import ast

csv_force = "force.csv"
csv_strain = "strain.csv"
csv_dspl = "displacement.csv"
csv_distance = "distance.csv"

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()


from dynamixel_sdk import * # Uses Dynamixel SDK library

######################
    # GUI
######################
# GUI for plots
def draw_figure(canvas, figure):
    figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
    figure_canvas_agg.draw()
    figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
    return figure_canvas_agg

# Creating the UI elements and layout
sg.theme('Default1')
charac_text = sg.Text("Characterization", font=("Computer Modern Roman", 12, "bold"))
charac_force = sg.Radio("Force", "charac", key = "charac_force", enable_events=True)
charac_dspl = sg.Radio("Displacement", "charac", key = "charac_dspl", enable_events=True)
charac_discrete = sg.Radio("Discrete Force-Strain", "charac", key = "charac_discrete", circle_color = "Blue", text_color="Blue", enable_events=True)

profile_text = sg.Text("Measurement profile", font=("Helvetica", 10, "bold"), key='profile_text', visible=True)
profile_list = sg.Combo(["Constant", "Sine wave", "Ramp"], key = 'profile_list', enable_events=True, disabled=True, visible=True)

force_text = sg.Text("Max Force [N]", key = 'force_text',  font=("Helvetica", 10, "bold"), visible=True)
force_slider = sg.Slider(range=(0.5,20), default_value=0.5, resolution=0.1, orientation='horizontal', key = 'force_slider', enable_events=True, disabled = True, visible=True)
discrete_text_1 = sg.Text("Desired discrete force array [N]", key = 'discrete_text_1',  font=("Helvetica", 10, "bold"), visible=True)
discrete_text_2 = sg.Text("-Form: [0.00, x.xx, y.yy, z.zz]\n-max force <= 20N\n-step size between 0.1 and 2.0N\n-only increasing values", size=(30, 5), key = 'discrete_text_2', visible=True)
discrete_array_box = sg.Input(size=(30,0), disabled = True, key='discrete_array_box', enable_events=True, visible=True)


dspl_text = sg.Text("Max Displacement [mm]", key = 'dspl_text',  font=("Helvetica", 10, "bold"), visible=True)
dspl_slider = sg.Slider(range=(0.1,15.0), default_value=0.1, resolution=0.1, orientation='horizontal', key = 'dspl_slider', enable_events = True, disabled = True, visible=True)

voltage_text = sg.Text("Applied Voltage [kV]", key = 'voltage_text',  font=("Helvetica", 10, "bold"), visible=True)
voltage_slider = sg.Slider(range=(0,10.0), default_value=0.1, resolution=0.1, orientation='horizontal', key = 'voltage_slider', enable_events = True, disabled = True, visible=True)
duration_text = sg.Text("Measurement duration [s]", key = 'duration_text',  font=("Helvetica", 10, "bold"), visible=True)
duration_box = sg.Input(size=(3,0), disabled = True, key='duration_box', enable_events=True, visible=True)
length_text = sg.Text("Initial HASEL length [mm]", key = 'length_text',  font=("Helvetica", 10, "bold"), visible=True)
length_box = sg.Input(size=(3,0), disabled = True, key='length_box', enable_events=True, visible=True)


button_ready = sg.Button("Ready", key='ready', disabled = True, enable_events=True)
button_start = sg.Button("Start Measurement", button_color='white smoke', key='Start Measurement', disabled = True, enable_events=True)
button_save = sg.Button("Save Figure", disabled = True, enable_events=True)
button_close = sg.Button("Close")


preview_plot = sg.Canvas(key='preview_plot')
live_plot = sg.Canvas(key='live_plot')
left_Column = [[sg.Frame("", layout = [[charac_text], [charac_force, charac_dspl, charac_discrete], [profile_text], [profile_list], [force_text], [force_slider], [dspl_text], [dspl_slider], [discrete_text_1], [discrete_text_2], [discrete_array_box], [voltage_text, voltage_slider], [duration_text, duration_box], [length_text, length_box],])], [button_ready], [button_start], [button_save, button_close]]
middle_Column = [[sg.Text("Profile preview", font=("Helvetica", 12, "bold"))], [preview_plot]]
right_Column = [[sg.Text("Measured data", font=("Helvetica", 12, "bold"))], [live_plot]]

layout = [[sg.Column(left_Column), sg.Column(middle_Column), sg.Column(right_Column)]]

# Setting up the data plot
goal_force_array = []
present_force_array = []
goal_dspl_array = []
present_dspl_array = []
time_stamps = []
empty_array = []
strain_array = []
distance_array = []
target_array = []
intern_target_array = []


# For Ready Button Update
ready_index = 0

# Save figure function
def save_figure():
    global rightFig
    save_path = sg.popup_get_file('Save Figure', save_as=True, file_types=(('PNG Files', '*.png'),))
    if save_path:
        rightFig.savefig(save_path)
        sg.popup(f'Figure saved as: {save_path}')

def check_array(user_input):
    global intern_target_array, target_array
    checkApproved = True
    max_force = 0.00
    target_array = ast.literal_eval(user_input)

    # Check whether its a list at all
    if not isinstance(target_array, list):
        print("Not a valid array.")
        sg.popup("Invalid array input. Check requirements.")
        intern_target_array = []
        return False

    # Check whether values are valid
    for k in range(len(target_array)):
        if k == 0:
            if target_array[k] != 0.00:
                print("Wrong zero.")
                checkApproved = False
            intern_target_array.append(-0.10)
        else:
            curr_elem = target_array[k]
            prev_elem = target_array[k-1]
            if curr_elem > 20 or curr_elem <= prev_elem + 0.09 or curr_elem > prev_elem + 2:    # Starting at 0N, max not bigger than 20N, always increasing, step not too big or too small
                print("Value error.")
                checkApproved = False
            intern_target_array.append(target_array[k] + 0.03)
            max_force = target_array[k]

    # Invalid array
    if not checkApproved:
        print("Invalid.")
        sg.popup("Invalid array input. Check requirements.")
        intern_target_array = []
        return False

    # Valid array     
    print("Target array:", target_array)
    print("Intern target array: ", intern_target_array)
    update_preview_plot('charac_discrete', 'Discrete', max_force)
    return True
    
    
######################
    # SETUP OF INSTRUMENTS
######################

ADDR_OPERATING_MODE         = 11
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_VELOCITY          = 104
ADDR_VELOCITY_LIMIT         = 44

BAUDRATE                    = 57600
PROTOCOL_VERSION            = 2.0
DXL_ID                      = 1
DEVICENAME                  = 'COM3'
TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque


# Setup PID controller
Kp = 32
Ki = 0
Kd = 20

target = 0.47
previous_error = 0
integral = 0

voltage_mapping_factor = 2000   # CHANGE

unactuated_distance = 0.00
initial_length = 1.00

# Serial communication with load cell via HX711
ser = serial.Serial('COM5', 57600)

# Communication with DAQ
def read_distance():
    with nidaqmx.Task() as task:
        task.ai_channels.add_ai_voltage_chan("Dev1/ai0")
        reading = task.read() * 4 + 30
        return round(reading,2)

def write_voltage(desired_voltage): # TREK 20/20C: 1V DAQ input = 2000V desired output -> voltage_mapping_factor = 2000, 'desired' should be in [V]
    daq_voltage = round(float(desired_voltage / voltage_mapping_factor),2)
    with nidaqmx.Task() as task:
        task.ao_channels.add_ao_voltage_chan("Dev1/ao0")
        task.write(daq_voltage, auto_start=True)

# Calculates relative strain and absolute distance difference
def get_strain():
    return round((read_distance() - unactuated_distance) * 100 / initial_length,2)

def get_distance_difference():
    return round(starting_distance - read_distance(),2)

# Clean console ouput
def print_values(target, force, vel, starting_distance, distance, distance_difference):
    print("Target: ", target)
    print("Force: ", force)
    print("Vel: ", vel)
    print("Starting distance: ", starting_distance)
    print("Current distance: ", distance)
    print("Distance difference: ", distance_difference)


    sys.stdout.write('\033[F')  
    sys.stdout.write('\033[K')
    sys.stdout.write('\033[F')  
    sys.stdout.write('\033[K')
    sys.stdout.write('\033[F')  
    sys.stdout.write('\033[K')
    sys.stdout.write('\033[F')  
    sys.stdout.write('\033[K')
    sys.stdout.write('\033[F')  
    sys.stdout.write('\033[K')
    sys.stdout.write('\033[F')  
    sys.stdout.write('\033[K')

# Pause script without interrupting serial communication and affecting data readings
def dead_force():
    start_time = time.time()
    while time.time()-start_time < 2:
        if ser.in_waiting > 0:
            dead_force = ser.readline().decode('utf-8').rstrip()

# Initialize DYNAMIXEL
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
if portHandler.openPort():
    print("Succeeded to open the port")
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_VELOCITY_LIMIT, 1023)

# Set operating mode and enable torque
def start_servo(operating_mode):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, operating_mode) # 1 = velocity control, 3 = position control, 4 = extended position control
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    print("Dynamixel has been successfully connected")

# Disable torque and stop servo
def stop_servo():
    # Disable Dynamixel Torque (DYNAMIXEL PRESET)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    print("Dynamixel has been successfully disconnected")


######################
    # DEFINITION WRITE COMMAND SERVO
######################

# Write goal velocity
def write_vel(vel):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, vel)

######################
    # DEFINITIONS CHARACTERIZATION PROFILE CONTROLS
######################  
abort_mission = False  
measurement_type = "No"     
def calibration():
    global integral, old_vel, previous_error, starting_distance, abort_mission
    reached = False
    while 1:
        if ser.in_waiting > 0:
            if keyboard.is_pressed('esc'):
                print("CALIBRATION ABORTED")
                abort_mission = True
                break

            # Read force data
            force = ser.readline().decode('utf-8').rstrip()   
            while force == '':
                force = ser.readline().decode('utf-8').rstrip()
            force = float(force) 
            error = target - force
            
            # Calculate PID parameters and write new velocity
            proportional = Kp * error
            integral += Ki * error
            derivative = Kd * (error - previous_error)

            new_vel = max(min(int(proportional + integral + derivative), 20), -20)
            if new_vel != old_vel:
                write_vel(new_vel)

            print_values(0.50, force, new_vel, starting_distance, read_distance(), get_distance_difference())
            old_vel = new_vel
            previous_error = error

            # Check for desired calibration state
            if abs(force - 0.50) <= 0.015:
                if not reached:
                    reached = True
                    hold_time = time.time()
                if time.time() - hold_time >= 1:
                    starting_distance = read_distance()
                    print("CALIBRATION FINISHED")
                    write_vel(0)
                    break
            else:
                reached = False

def constant_force(max_force, duration):
    global integral, old_vel, previous_error, target
    target = max_force
    start_time = time.time()
    while 1:
        if ser.in_waiting > 0:
            if keyboard.is_pressed('esc'):
                print("MEASUREMENT ABORTED")
                break
            
            # Read force data
            force = ser.readline().decode('utf-8').rstrip()   
            while force == '':
                force = ser.readline().decode('utf-8').rstrip()
            force = float(force) 
            error = target - force
            
            # Calculate PID parameters and write new velocity
            proportional = Kp * error
            integral += Ki * error
            derivative = Kd * (error - previous_error)

            new_vel = max(min(int(proportional + integral + derivative), 20), -20)
            if new_vel != old_vel:
                write_vel(new_vel)

            print_values(target, force, new_vel, starting_distance, read_distance(), get_distance_difference())
            old_vel = new_vel
            previous_error = error

            # Save data in array and check for goal
            #goal_force_array.append(target)
            present_force_array.append(force)
            present_dspl_array.append(get_distance_difference())
            time_stamps.append(time.time()-start_time)
            if time.time()-start_time > duration:
                print("MEASUREMENT DONE")
                break

def constant_dspl(max_dspl, duration):
    global integral, old_vel, previous_error, target
    target = starting_distance - max_dspl
    start_time = time.time()
    while 1:
        if ser.in_waiting > 0:
            if keyboard.is_pressed('esc'):
                print("MEASUREMENT ABORTED")
                break
            
            # Read force data
            force = ser.readline().decode('utf-8').rstrip()   
            while force == '':
                force = ser.readline().decode('utf-8').rstrip()
            force = float(force) 

            error = target - read_distance()
            
            # Calculate PID parameters and write new velocity
            proportional = Kp * error
            integral += Ki * error
            derivative = Kd * (error - previous_error)

            new_vel = -max(min(int(proportional + integral + derivative), 20), -20)
            if new_vel != old_vel:
                write_vel(new_vel)

            print_values(target, force, new_vel, starting_distance, read_distance(), get_distance_difference())
            old_vel = new_vel
            previous_error = error

            # Save data in array and check for goal
            #goal_dspl_array.append(max_dspl)
            present_dspl_array.append(get_distance_difference())
            present_force_array.append(force)
            time_stamps.append(time.time()-start_time)
            if time.time()-start_time > duration:
                print("MEASUREMENT DONE")
                break

def ramp_force(max_force):
    global integral, old_vel, previous_error, target
    target = 0.50
    start_time = time.time()
    while 1:
        if ser.in_waiting > 0:
            if keyboard.is_pressed('esc'):
                print("MEASUREMENT ABORTED")
                break
            
            # Read force data
            force = ser.readline().decode('utf-8').rstrip()   
            while force == '':
                force = ser.readline().decode('utf-8').rstrip()
            force = float(force) 
            error = target - force
            
            # Calculate PID parameters and write new velocity
            proportional = Kp * error
            integral += Ki * error
            derivative = Kd * (error - previous_error)

            new_vel = max(min(int(proportional + integral + derivative), 20), -20)
            if new_vel != old_vel:
                write_vel(new_vel)

            print_values(target, force, new_vel, starting_distance, read_distance(), get_distance_difference())
            old_vel = new_vel
            previous_error = error

            # Save data in array and check for goal
            #goal_force_array.append(target)
            present_force_array.append(force)
            present_dspl_array.append(get_distance_difference())
            time_stamps.append(time.time()-start_time)
            if force >= max_force:
                print("MEASUREMENT DONE")
                break
            target += 0.01
            target = round(target,2)

def ramp_dspl(max_dspl):
    global integral, old_vel, previous_error, target
    target = starting_distance
    start_time = time.time()
    while 1:
        if ser.in_waiting > 0:
            if keyboard.is_pressed('esc'):
                print("MEASUREMENT ABORTED")
                break
            
            # Read force and displacement data
            force = ser.readline().decode('utf-8').rstrip()   
            while force == '':
                force = ser.readline().decode('utf-8').rstrip()
            force = float(force)

            error = target - read_distance()
            
            # Calculate PID parameters and write new velocity
            proportional = Kp * error
            integral += Ki * error
            derivative = Kd * (error - previous_error)

            new_vel = -max(min(int(proportional + integral + derivative), 20), -20)
            if new_vel != old_vel:
                write_vel(new_vel)

            print_values(target, force, new_vel, starting_distance, read_distance(), get_distance_difference())
            old_vel = new_vel
            previous_error = error

            # Save data in array and check for goal
            #goal_dspl_array.append(abs(starting_distance - target))
            present_dspl_array.append(get_distance_difference())
            present_force_array.append(force)
            time_stamps.append(time.time()-start_time)
            if get_distance_difference() >= max_dspl:
                print("MEASUREMENT DONE")
                break
            target -= 0.01
            target = round(target,2)

def sine_force(max_force, duration):
    global integral, old_vel, previous_error, target
    angle = 180
    start_time = time.time()
    while 1:
        if ser.in_waiting > 0:
            if keyboard.is_pressed('esc'):
                print("MEASUREMENT ABORTED")
                break
            
            # Read force data
            force = ser.readline().decode('utf-8').rstrip()   
            while force == '':
                force = ser.readline().decode('utf-8').rstrip()
            force = float(force) 

            target = 0.5 + ((math.cos(math.radians(angle)) * ((max_force-0.5)/2)) + ((max_force-0.5)/2))
            error = target - force
            
            # Calculate PID parameters and write new velocity
            proportional = Kp * error
            integral += Ki * error
            derivative = Kd * (error - previous_error)

            new_vel = max(min(int(proportional + integral + derivative), 50), -50)
            if new_vel != old_vel:
                write_vel(new_vel)

            print_values(target, force, new_vel, starting_distance, read_distance(), get_distance_difference())
            old_vel = new_vel
            previous_error = error

            # Save data in array and check for goal
            #goal_force_array.append(target)
            present_force_array.append(force)
            present_dspl_array.append(get_distance_difference())
            time_stamps.append(time.time()-start_time)

            if time.time()-start_time > duration:
                print("MEASUREMENT DONE")
                break
            angle += 4
            
def sine_dspl(max_dspl, duration):
    global integral, old_vel, previous_error, target
    angle = 180
    start_time = time.time()
    Kp = 60
    while 1:
        if ser.in_waiting > 0:
            if keyboard.is_pressed('esc'):
                print("MEASUREMENT ABORTED")
                break
            
            # Read force data
            force = ser.readline().decode('utf-8').rstrip()   
            while force == '':
                force = ser.readline().decode('utf-8').rstrip()
            force = float(force) 

            target = starting_distance - ((math.cos(math.radians(angle)) * (max_dspl/2)) + (max_dspl/2))
            error = target - read_distance()
            
            # Calculate PID parameters and write new velocity
            proportional = Kp * error
            integral += Ki * error
            derivative = Kd * (error - previous_error)

            new_vel = -max(min(int(proportional + integral + derivative), 20), -20)
            if new_vel != old_vel:
                write_vel(new_vel)

            print_values(target, force, new_vel, starting_distance, read_distance(), get_distance_difference())
            old_vel = new_vel
            previous_error = error

            # Save data in array and check for goal
            #goal_dspl_array.append(((math.cos(math.radians(angle)) * (max_dspl/2)) + (max_dspl/2)))
            present_dspl_array.append(get_distance_difference())
            present_force_array.append(force)
            time_stamps.append(time.time()-start_time)
            
            if time.time()-start_time > duration:
                print("MEASUREMENT DONE")
                break
            angle += 4

def discrete_measurement(voltage):
    global integral, old_vel, previous_error, target, unactuated_distance, intern_target_array, target_array
    target = 0.30
    i = 0

    reached_turning_point = False
    reached_unactuated_distance = False
    reached = False
    while 1:
        if ser.in_waiting > 0:
            # Keyboard inputs
            if keyboard.is_pressed('esc'):  # Stops Measurement process
                print("MEASUREMENT ABORTED")
                write_vel(0)
                write_voltage(0)
                break

            force = ser.readline().decode('utf-8').rstrip()  # Daten vom Arduino empfangen und dekodieren 
            while force == '':
                force = ser.readline().decode('utf-8').rstrip()
            force = float(force)

            error = target - force
            
            proportional = Kp * error
            integral += Ki * error
            derivative = Kd * (error - previous_error)

            new_vel = max(min(int(proportional + integral + derivative), 20), -20)
            if new_vel != old_vel:
                write_vel(new_vel)

            print_values(target_array[i], force, new_vel, unactuated_distance, read_distance(), get_strain())
            old_vel = new_vel
            previous_error = error

            if force > 0.25 and reached_turning_point == False:
                reached_turning_point = True
                target = intern_target_array[i]
                print("Reached Turning point")
            if reached_turning_point == True and abs(force) <= 0.01 and reached_unactuated_distance == False:
                if not reached:
                    reached = True
                    hold_time = time.time()
                if time.time() - hold_time >= 1:
                    reached_unactuated_distance = True
                    unactuated_distance = read_distance()
                    distance_array.append(unactuated_distance)
                    print("CALIBRATION FINISHED")
                    reached = False
                    write_voltage(voltage)
                    print("Voltage applied")
            elif abs(force - target_array[i]) <= 0.015 and reached_unactuated_distance == True:
                if not reached:
                    reached = True
                    hold_time = time.time()
                if time.time() - hold_time >= 1:
                    strain_array.append(get_strain())
                    distance_array.append(read_distance())
                    present_force_array.append(target_array[i])
                    empty_array.append(0)
                    i += 1
                    if i == len(target_array) or strain_array[i-1] < 0:
                        print("MEASUREMENT FINISHED")
                        break
                    target = intern_target_array[i]
            else:
                reached = False


######################
    # START GUI - MAIN SOFTWARE
######################

# Polarity Switch upon start
print("Polarity Switch Start")
starting_distance = 0.00
write_voltage(0)
dead_force()
write_voltage(-4000)
dead_force()
write_voltage(0)
dead_force()
print("Polarity Switch Finished")

# Create the window
window = sg.Window("HASEL Test bench", layout, finalize=True)

# Update preview plot according to characterization and profile choice
def update_preview_plot(charac, profile, amplitude):
    global figAgg1, leftAx, leftFig
    if values['duration_box'] == '':
        duration = 11
    else:
        duration = int(values['duration_box']) + 1
    xData = np.arange(0, duration, 1)
    leftAx.cla()

    # Updating axes
    if charac == 'charac_force':
        leftAx.set_xlabel("Time")
        leftAx.set_ylabel("Force [N]")
        leftAx.set_title("Force profile")
    elif charac == 'charac_dspl':
        leftAx.set_xlabel("Time")
        leftAx.set_ylabel("Displacement [mm]")
        leftAx.set_title("Displacement profile")
    elif charac == 'charac_discrete':
        leftAx.set_xlabel("Strain [%]")
        leftAx.set_ylabel("Force [N]")
        leftAx.set_title("Discrete Value Curve Sweeping")
        profile = 'Discrete'
    # Constant profile
    if profile == 'Constant':
        yData = np.full((duration,), amplitude)
        leftAx.plot(xData, yData, color = 'violet')
        leftAx.set_xlim(0,len(xData)-1)
        leftAx.set_ylim(0,amplitude*2)
    # Sine profile
    elif profile == 'Sine wave':
        xData = np.linspace(0, duration, 1000)
        if charac == 'charac_force':
            yData = (((amplitude-0.5)/2) * np.cos((2/3 * np.pi * xData)-np.pi) + ((amplitude-0.5)/2)) + 0.5
        elif charac == 'charac_dspl':
            yData = ((amplitude/2) * np.cos((2/3 * np.pi * xData)-np.pi) + (amplitude/2))
        leftAx.plot(xData,yData,color='violet')
        leftAx.set_xlim(0,duration)
        leftAx.set_ylim(0,amplitude*2)
    # Ramp profile
    elif profile == 'Ramp':
        xData = np.linspace(0, duration, 100)
        if charac == 'charac_force':
            yData = (((amplitude-0.5) / duration) * xData) + 0.5
        elif charac == 'charac_dspl':
            yData = ((amplitude / duration) * xData)
        leftAx.plot(xData, yData, color = 'violet')
        leftAx.set_xlim(0,duration)
        leftAx.set_ylim(0,amplitude*2)
    # Discrete measurement
    elif profile == 'Discrete':
        xData = [6.85, 4.92, 4.04, 2.94, 2.1, 1.59, 1.19, 0.92, 0.79, 0.67, 0.58, 0.52]
        yData = [0.01, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0 , 9.0, 10.0]
        leftAx.plot(xData, yData, color = 'orange', marker = 'x', markeredgecolor = 'black', linewidth = 2)
        leftAx.set_xlim(0,8)
        leftAx.set_ylim(0,amplitude + 0.1)
        

    # Redrawing plot
    leftAx.set_xticks([])
    leftAx.grid(True,which='both')
    figAgg1.draw()

def update_live_plot(yArray1, yArray2, xArray, mode):   # mode 0 = force, mode 1 = dspl, mode 2 = discrete 
    global figAgg2, ax1, ax2, rightFig, line1, line2

    if mode == 0:
        ax1.set_title("Dynamic Measurement @ " + str(values['voltage_slider']) + " kV")
        ax1.set_xlabel('Time [s]')
        ax1.set_ylabel('Force [N]')
        ax2.set_ylabel('Displacement [mm]')
        line1.set_label('Force')
        line2.set_label('Displacement')
        line1.set_color('red')
        line2.set_color('blue')
        labels = [line.get_label() for line in lines]
        ax1.legend(lines, labels)
        line1.set_data(xArray, yArray1)
        line2.set_data(xArray, yArray2)
    elif mode == 1:
        ax1.set_title("Dynamic Measurement @ " + str(values['voltage_slider']) + " kV")
        ax1.set_xlabel('Time [s]')
        ax1.set_ylabel('Displacement [mm]')
        ax2.set_ylabel('Force [N]')
        line1.set_label('Displacement')
        line2.set_label('Force')
        line1.set_color('blue')
        line2.set_color('red')
        labels = [line.get_label() for line in lines]
        ax1.legend(lines, labels)
        line1.set_data(xArray, yArray1)
        line2.set_data(xArray, yArray2)
    elif mode == 2:
        ax1.set_title("Discrete Measurement @ " + str(values['voltage_slider']) + " kV")
        ax1.set_xlabel('Strain [%]')
        ax1.set_ylabel('Force [N]')
        ax2.set_ylabel('')
        line1.set_color('orange')
        line1.set_marker('x')
        line1.set_markeredgecolor('black')
        line1.set_linewidth(2)
        ax1.set_xlim(0,max(xArray)+1)
        line1.set_data(xArray, yArray1)
        ax2.set_yticks([])

    ax1.relim()
    ax1.autoscale_view()
    ax1.set_ylim(0, max(yArray1) * 1.5)
    if mode != 2:
        ax2.set_ylim(0, max(yArray2) * 1.5)
    figAgg2.draw()
    window.refresh()

# Initializing empty plot upon starting script
leftFig = plt.figure()
leftAx = leftFig.add_subplot(111)
leftAx.plot()
leftAx.set_xticks([])
leftAx.grid(True,which='both')
figAgg1 = draw_figure(window['preview_plot'].TKCanvas, leftFig)

rightFig, ax1 = plt.subplots()
ax2 = ax1.twinx()
ax1.grid(True, which='both')
line1, = ax1.plot([], [])
line2, = ax2.plot([], [])
lines = [line1, line2]
figAgg2 = draw_figure(window['live_plot'].TKCanvas, rightFig)


# Start script and open GUI
while 1:
    event, values = window.read()
    # Close window
    if event == "Close" or event == sg.WIN_CLOSED:
        break

    # Dynamic Characterization options
    elif event == 'charac_force' or event == 'charac_dspl':
        window['profile_text'].Update(visible = True)
        window['profile_list'].Update(disabled = False, visible = True)
        window['voltage_text'].Update(visible = True)
        window['voltage_slider'].Update(disabled = False, visible = True)
        window['duration_text'].Update(visible = True)
        window['duration_box'].Update(disabled = False, visible = True)
        window['length_text'].Update(visible = False)
        window['length_box'].Update(disabled = True, visible = False)
        window['ready'].update(disabled = False)
        if event == 'charac_force':
            window['force_text'].Update(visible = True)
            window['force_slider'].Update(disabled = False, visible = True)
            window['discrete_text_1'].Update(visible = False)
            window['discrete_text_2'].Update(visible = False)
            window['discrete_array_box'].Update(disabled = True, visible = False)
            window['dspl_text'].Update(visible = False)
            window['dspl_slider'].Update(disabled = True, visible = False)
            update_preview_plot('charac_force', values['profile_list'], values['force_slider'])
            
        elif event == 'charac_dspl':
            window['force_text'].Update(visible = False)
            window['force_slider'].Update(disabled = True, visible = False)
            window['discrete_text_1'].Update(visible = False)
            window['discrete_text_2'].Update(visible = False)
            window['discrete_array_box'].Update(disabled = True, visible = False)
            window['dspl_text'].Update(visible = True)
            window['dspl_slider'].Update(disabled = False, visible = True)
            update_preview_plot('charac_dspl', values['profile_list'], values['dspl_slider'])
            
    # Discrete Characterization options
    elif event == 'charac_discrete':
        window['profile_text'].Update(visible = False)
        window['profile_list'].Update(disabled = True, visible = False)
        window['voltage_text'].Update(visible = True)
        window['voltage_slider'].Update(disabled = False, visible = True)
        window['duration_text'].Update(visible = False)
        window['duration_box'].Update(disabled = True, visible = False)
        window['length_text'].Update(visible = True)
        window['length_box'].Update(disabled = False, visible = True)
        window['ready'].update(disabled = False)
        window['force_text'].Update(visible = False)
        window['force_slider'].Update(disabled = True, visible = False)
        window['discrete_text_1'].Update(visible = True)
        window['discrete_text_2'].Update(visible = True)
        window['discrete_array_box'].Update(disabled = False, visible = True)
        window['dspl_text'].Update(visible = False)
        window['dspl_slider'].Update(disabled = True, visible = False)
        #update_preview_plot('charac_discrete', 'Discrete', values['discrete_force_slider'])

    # Characterization profiles
    elif event == 'profile_list':
        if values['charac_force']:
            temp1 = 'charac_force'
            temp2 = values['force_slider']
        else:
            temp1 = 'charac_dspl'
            temp2 = values['dspl_slider']
        if values['profile_list'] == "Constant":
            update_preview_plot(temp1, 'Constant', temp2)
        elif values['profile_list'] == "Sine wave":
            update_preview_plot(temp1, 'Sine wave', temp2)
        elif values['profile_list'] == "Ramp":
            update_preview_plot(temp1, 'Ramp', temp2)

    # Change slider values and update preview plot
    elif event == 'force_slider' or event == 'dspl_slider':
        if values['charac_force']:
            temp1 = 'charac_force'
            temp2 = values['force_slider']
        elif values['charac_dspl']:
            temp1 = 'charac_dspl'
            temp2 = values['dspl_slider']
        update_preview_plot(temp1, values['profile_list'], temp2)
    

    # Lock input parameters and prepare servo for measurement
    elif event == 'ready':
        if (values['charac_discrete'] and (values['length_box'] != "" and check_array(str(values['discrete_array_box'])))) or (values['profile_list'] != "" and values['duration_box'] != ""):
            if ready_index == 0:
                window['ready'].update(text="Unready")
                window['charac_force'].Update(disabled=True)
                window['charac_dspl'].Update(disabled=True)
                window['charac_discrete'].Update(disabled=True)
                window['profile_list'].Update(disabled=True)
                window['force_slider'].Update(disabled=True)
                window['discrete_array_box'].Update(disabled=True)
                window['dspl_slider'].Update(disabled=True)
                window['voltage_slider'].Update(disabled=True)
                window['duration_box'].Update(disabled=True)
                window['length_box'].Update(disabled = True)
                window['Start Measurement'].Update(disabled=False, button_color='DarkOliveGreen3')
                start_servo(1)
                ser.write(str(1).encode())  # Tares load cell
                dead_force()
                sg.popup("Please verify your parameter inputs. UNREADY if you want to change parameters. START MEASUREMENT if you want to start.")

                ready_index = 1
            else:
                window['ready'].update(text="Ready")
                window['charac_force'].Update(disabled=False)
                window['charac_dspl'].Update(disabled=False)
                window['charac_discrete'].Update(disabled=False)
                window['profile_list'].Update(disabled=False)
                window['force_slider'].Update(disabled=False)
                window['discrete_array_box'].Update(disabled=False)
                window['dspl_slider'].Update(disabled=False)
                window['voltage_slider'].Update(disabled=False)
                window['duration_box'].Update(disabled=False)
                window['length_box'].Update(disabled = False)
                window['Start Measurement'].Update(disabled=True, button_color='white smoke')
                stop_servo()

                ready_index = 0
        else:
            sg.popup("Please adjust required settings.")

    # Start selected measurement upon pressing button
    elif event == "Start Measurement":
        window['ready'].Update(disabled=True)
        window['Start Measurement'].Update(disabled=True, button_color='white smoke')
        old_vel = 0
        write_vel(old_vel)
        applied_voltage = values['voltage_slider']
        if values['charac_force']:
            measurement_type = 'Force'
        elif values['charac_dspl']:
            measurement_type = 'Displacement'
        elif values['charac_discrete']:
            measurement_type = 'Discrete force-strain'


        # Dynamic force profile measurements
        if values['charac_force']:
            write_voltage(float(values['voltage_slider']) * 1000)
            dead_force()
            calibration()
            if abort_mission:
                break
            dead_force()
            if values['profile_list'] == 'Ramp':
                ramp_force(float(values['force_slider']))
            elif values['profile_list'] == 'Constant':
                constant_force(float(values['force_slider']), float(values['duration_box']))
            elif values['profile_list'] == 'Sine wave':
                sine_force(float(values['force_slider']), float(values['duration_box']))
          
            update_live_plot(present_force_array, present_dspl_array, time_stamps, 0)

        # Dynamic displacement profile measurements
        elif values['charac_dspl']:
            write_voltage(float(values['voltage_slider']) * 1000)
            dead_force()
            calibration()
            if abort_mission:
                break
            dead_force()
            if values['profile_list'] == 'Ramp':
                ramp_dspl(float(values['dspl_slider']))
            elif values['profile_list'] == 'Constant':
                constant_dspl(float(values['dspl_slider']), float(values['duration_box']))
            elif values['profile_list'] == 'Sine wave':
                sine_dspl(float(values['dspl_slider']), float(values['duration_box']))
        
            update_live_plot(present_dspl_array, present_force_array, time_stamps, 1)

        # Discrete Force-Strain curve sweeping
        elif values['charac_discrete']:
            initial_length = float(values['length_box'])
            discrete_measurement(float(values['voltage_slider'])*1000)

            update_live_plot(present_force_array, empty_array, strain_array, 2)

        # Remove voltage after measurement
        write_voltage(0)
        print("VOLTAGE REMOVED")   
        write_vel(0)
        window['Save Figure'].Update(disabled=False)

    # Save figure in the end
    elif event == 'Save Figure':
        save_figure()

write_vel(0)
stop_servo()

applied_voltage = values['voltage_slider']
measurement_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
txt_content = f"Date: {measurement_time}, Applied Voltage: {applied_voltage} kV, {measurement_type} Characterization type"
if measurement_type == 'Discrete force-strain':
    txt_content += f", Initial HASEL length [mm]: {initial_length}"


with open('experiment_log.txt', 'w') as file:
    file.write(txt_content)
with open(csv_force, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(present_force_array)
with open(csv_strain, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(strain_array)
with open(csv_dspl, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(present_dspl_array)
with open(csv_distance, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(distance_array)

write_voltage(0)
print("VOLTAGE DEFINITELY REMOVED")
ser.close()
portHandler.closePort()
print("Bye")

window.close()
