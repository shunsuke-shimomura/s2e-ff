#
# Plot Satellite Relative Position on RTN frame
#
# arg[1] : read_file_tag : time tag for default CSV output log file. ex. 220627_142946
#

#
# Import
#
# plots
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# local function
from common import find_latest_log_tag
from common import read_3d_vector_from_csv, read_scalar_from_csv
# csv read
import pandas
# arguments
import argparse

#
# Read Arguments
#
aparser = argparse.ArgumentParser()

aparser.add_argument('--logs-dir', type=str, help='logs directory like "../../data/logs"', default='../../data/logs')
aparser.add_argument('--file-tag', type=str, help='log file tag like 220627_142946')
aparser.add_argument('--no-gui', action='store_true')

args = aparser.parse_args()

# log file path
path_to_logs = args.logs_dir

read_file_tag = args.file_tag
if read_file_tag == None:
  print("file tag does not found. use latest the latest log file.")
  read_file_tag = find_latest_log_tag(path_to_logs)

print("log: " + read_file_tag)

#
# CSV file name
#
read_file_name  = path_to_logs + '/' + 'logs_' + read_file_tag + '/' + read_file_tag + '_default.csv'

#
# Data read and edit
#
# Read S2E CSV
t = read_scalar_from_csv(read_file_name, "elapsed_time[s]")
d = read_scalar_from_csv(read_file_name, 'satellite1_position_from_satellite0_rtn_z[m]')
print(t)
# Add satellites if you need
pos = read_3d_vector_from_csv(read_file_name, 'satellite1_position_from_satellite0_rtn', 'm')
estimated = read_3d_vector_from_csv(read_file_name, 'kalman_filter_estimated_position_rtn', "m")
# Edit data if you need

#
# Plot
#
fig = plt.figure(figsize=(5,5))
ax = fig.add_subplot(111)
ax.set_title("distance")
ax.set_xlabel("time [s]")
ax.set_ylabel("distance [m]")

# Add plot settings if you need
#ax.set_xlim(-100, 100)
#ax.set_ylim(-100, 100)
#ax.set_zlim(-100, 100)

ax.plot(t,d,marker="o")

ax.legend()

fig = plt.figure(figsize=(5,5))
ax = fig.add_subplot(111)
ax.set_title("distance")
ax.set_xlabel("time [s]")
ax.set_ylabel("distance [m]")

# Add plot settings if you need
#ax.set_xlim(-100, 100)
#ax.set_ylim(-100, 100)
#ax.set_zlim(-100, 100)

ax.plot(t[0][3:],estimated[2][3:])
ax.plot(t[0][3:],pos[2][3:])

ax.legend()

fig = plt.figure(figsize=(5,5))
ax = fig.add_subplot(111)
ax.set_title("distance")
ax.set_xlabel("time [s]")
ax.set_ylabel("distance [m]")

# Add plot settings if you need
#ax.set_xlim(-100, 100)
#ax.set_ylim(-100, 100)
#ax.set_zlim(-100, 100)

ax.plot(t[0][3:],pos[2][3:]-estimated[2][3:])

ax.legend()

fig = plt.figure(figsize=(5,5))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Relative Position of Satellites in RTN frame")
ax.set_xlabel("Radial [m]")
ax.set_ylabel("Transverse [m]")
ax.set_zlabel("Normal [m]")
ax.plot(0,0,0, marker="*", c="green", markersize=10, label="Sat0")
ax.plot(pos[0],pos[1],pos[2], marker="x", linestyle='-', c="red", label="Sat1")

ax.legend()

if args.no_gui:
  plt.savefig(read_file_tag + "_relative_position_rtn.png")
else:
  plt.show()
