#!/usr/bin/env python
import os
import rospkg
os.chdir(os.path.join(rospkg.RosPack().get_path('moving_control')))
os.system("python compile.py build_ext --inplace")