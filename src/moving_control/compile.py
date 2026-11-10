from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
import os
import rospkg

# python compile.py build_ext --inplace
# cwd = os.path.abspath(os.path.join(os.getcwd(), os.pardir)) + '/scripts'

########################## moving_control ##########################
print('-----------------------------moving_control-----------------------------')
cwd = os.path.join(rospkg.RosPack().get_path('moving_control'))
if os.path.isdir(cwd + '/scripts'):
    cwd += '/scripts'
    # change directory to /release to call script from other directory
    os.chdir(os.path.join(rospkg.RosPack().get_path('moving_control'), 'release'))
    ext_modules = []
    ext_modules.append(Extension("moving_control_server", [cwd + "/moving_control_server.py"]))

    setup(
        name = 'moving_control',
        cmdclass = {'build_ext': build_ext},
        ext_modules = ext_modules
    )
else:
    print('{} is not exist -- IGNORE'.format(cwd))
