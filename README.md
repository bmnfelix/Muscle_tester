# Muscle_tester
Electrostatic muscle tester software v1.0
Bachelor Thesis - Felix Baumann - July 2023

The three main scripts are _muscle_tester.py_, _instrument_tester.py_ and _emergency_stop.py_

# Installation Guide (Windows)
1. Open powershell, change directory to _python_ folder [cd PATH\python]
2. Install _setup.py_ [python setup.py install (--user)]
3. Install all other necessary libraries [pip install LIBRARY]
4. Change COM ports if necessary
    - go to Device Manager and check _Arduino_ and _U2D2_ ports [COM*]
    - in _muscle_tester.py_, change 'DEVICENAME = 'COM*'' for _U2D2_ and 'ser = serial.Serial('COM*', 57600)' for _Arduino_
