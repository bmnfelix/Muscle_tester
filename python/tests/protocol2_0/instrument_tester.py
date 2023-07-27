import nidaqmx
import keyboard
import serial
import time
import sys


ser = serial.Serial('COM5', 57600)

voltage_index = 1
initial_length = 146         # Measure in mm, upper border - lower border pouch

def read_distance():
    with nidaqmx.Task() as task:
        task.ai_channels.add_ai_voltage_chan("Dev1/ai0")
        reading = task.read() * 4 + 30
        return round(reading,2)

def get_strain():
    return round((read_distance() - unactuated_distance) * 100 / initial_length,2)

def write_voltage(voltage): # 1V = 2kV output
    with nidaqmx.Task() as task:
        task.ao_channels.add_ao_voltage_chan("Dev1/ao0")
        task.write(voltage, auto_start=True)

def print_values(force, unactuated_distance, distance, strain, voltage_index):
    print("Force: ", force)
    print("Unactuated distance: ", unactuated_distance)
    print("Current Distance: ", distance)
    print("Strain: ", strain)
    print("Voltage: ", not voltage_index)

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

write_voltage(0)

unactuated_distance = read_distance()


print("press f to apply a voltage, press esc to exit")
while 1:
    if keyboard.is_pressed('f'):
        time.sleep(0.3)
        if voltage_index:
            write_voltage(2.5)
            voltage_index = 0
        else:
            write_voltage(0)
            voltage_index = 1
    elif keyboard.is_pressed('h'):
        ser.write(str(1).encode())
    elif keyboard.is_pressed('esc'):
        break

    force = ser.readline().decode('utf-8').rstrip()  # Daten vom Arduino empfangen und dekodieren 
    while force == '':
        force = ser.readline().decode('utf-8').rstrip()
    force = float(force)

    print_values(force, unactuated_distance, read_distance(), get_strain(), voltage_index)

write_voltage(0)
print("Voltage is OFF")
ser.close()






