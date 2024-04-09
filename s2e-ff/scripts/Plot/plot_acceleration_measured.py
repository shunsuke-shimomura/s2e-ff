import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# local function
from common import find_latest_log_tag
from common import read_3d_vector_from_csv,read_scalar_from_csv
# csv read
import pandas
# arguments
import argparse

aparser = argparse.ArgumentParser()

aparser.add_argument('--logs-dir', type=str, help='logs directory like "../../data/logs"', default='../../data/logs')
aparser.add_argument('--file-tag', type=str, help='log file tag like 220627_142946')
aparser.add_argument('--no-gui', action='store_true')

args = aparser.parse_args()

path_to_logs = args.logs_dir

read_file_tag = args.file_tag
if read_file_tag == None:
  print("file tag does not found. use latest the latest log file.")
  read_file_tag = find_latest_log_tag(path_to_logs)

print("log: " + read_file_tag)

read_file_name  = path_to_logs + '/' + 'logs_' + read_file_tag + '/' + read_file_tag + '_default.csv'
a_m = read_3d_vector_from_csv(read_file_name, 'RelativeAccelerationSensor_acceleration_i', 'm/s2')
a = read_3d_vector_from_csv(read_file_name, 'spacecraft_acceleration_i', 'm/s2')
t = read_scalar_from_csv(read_file_name, "elapsed_time[s]")

fig = plt.figure(figsize=(5,5))
ax = fig.add_subplot(111)
ax.set_title("acceleration")
ax.set_xlabel("time [s]")
ax.set_ylabel("acceleration [m]")

ax.plot(t[0][3:],a_m[0][3:])
ax.plot(t[0][3:],a[0][3:])

ax.legend()

if args.no_gui:
  plt.savefig(read_file_tag + "_relative_acceleration_rtn.png")
else:
  plt.show()