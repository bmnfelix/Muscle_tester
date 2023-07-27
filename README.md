# Muscle_tester
Electrostatic muscle tester software v1.0
Bachelor Thesis - Felix Baumann - July 2023

The three main scripts are _muscle_tester.py_, _instrument_tester.py_ and _emergency_stop.py_

found under python/tests/protocol2_0

# Installation Guide (Windows)
1. Open powershell, change directory to _python_ folder [cd PATH\python]
2. Install _setup.py_ [python setup.py install (--user)]
3. Install all other necessary libraries [(py -m) pip install LIBRARY]
4. Change COM ports if necessary
    - go to Device Manager and check _Arduino_ and _U2D2_ ports [COM*]
    - in all three main scripts listed above, change 'DEVICENAME = 'COM*'' for _U2D2_ and 'ser = serial.Serial('COM*', 57600)' for _Arduino_
  
# User guide
- always test all devices (HVPS, DAQ) before measurement by running _instrument_tester.py_, you can read data there and test the HVPS by pressing _f_
- in case of an emergency or after an aborted measurement, run _emergency_stop.py_ which disables the servo and remove HV
- the main tester script _muscle_tester.py_ will quickly switch polarity before the measurement, so do not touch electrostatic muscle after running the script
- GUI gives options to adjust parameters, _Ready_ button locks inputs, _Start Measurement_ button starts automated measurement
- _ESC_ button on keyboard aborts measurement
- save your figures with _Save Figure_ button, data arrays are saved automatically

  Contact _febaumann@ethz.ch_ if necessary
