import sys
from os.path import dirname, realpath
from rospkg import RosPack

project_path = RosPack().get_path('lift_help_predictor') + '/src/Lifting-from-the-Deep-release'

libs_dir_path = project_path + '/packages'

# Adding where to find libraries and dependencies
sys.path.append(libs_dir_path)
