#!/usr/bin/env python
import os
from os.path import expanduser

home = expanduser("~")
library_path = home + "/Arduino/libraries"
print(library_path)
os.chdir(library_path)

os.system("rm -rf ros_lib")
os.system("rosrun rosserial_arduino make_libraries.py .")
