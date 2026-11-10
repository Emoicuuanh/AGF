#!/usr/bin/env python
import serial
import serial.tools.list_ports

for p in serial.tools.list_ports.comports():
    # print(p.description)
    # print(p.device)
    # print(p.hwid)
    # print(p.interface)
    # print(p.location )
    # print(p.manufacturer )
    # print(p.name)
    # print(p.pid)
    # print(p.product)
    # print(p.serial_number)
    print(str(p.location) + ": " + str(p.device) + "    PID: " + str(p.pid))
print("-------------")
