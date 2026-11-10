import serial
import glob
import sys


def get_serial_ports(mark="A-Za-z"):
    """Lists serial port names

    :raises EnvironmentError:
        On unsupported or unknown platforms
    :returns:
        A list of the serial ports available on the system
    """
    if sys.platform.startswith("win"):
        ports = ["COM%s" % (i + 1) for i in range(256)]
    elif sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
        # this excludes your current terminal "/dev/tty"
        # ports = glob.glob('/dev/tty[A-Za-z]*') # all port
        ports = glob.glob("/dev/tty[{}]*".format(mark))
    elif sys.platform.startswith("darwin"):
        ports = glob.glob("/dev/tty.*")
    else:
        raise EnvironmentError("Unsupported platform")

    result = []
    # if ports:
    #     for port in ports:
    #         print(port)
    return ports
