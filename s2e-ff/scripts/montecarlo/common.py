# TODO read from s2e-core

import os
import numpy as np
from numpy.linalg import norm
import pandas

def find_latest_log_tag(logs_dir):
  dlist = sorted(os.listdir(logs_dir))
  latest_log = None
  for d in dlist:
    if os.path.isfile(logs_dir + d):
      continue
    if not d.startswith("logs_"):
      continue
    latest_log = d
  return latest_log[len("logs_"):] # remove `logs_` prefix

def normalize_csv_read_vector(vector):
  norm_v = norm(vector, axis=1)
  normalized_vector = vector / norm_v[:, None]
  return np.transpose(normalized_vector)

def read_3d_vector_from_csv(read_file_name, header_name, unit, skiprows=[1,1]):
  name_x = header_name + "_x" + '[' + unit + ']'
  name_y = header_name + "_y" + '[' + unit + ']'
  name_z = header_name + "_z" + '[' + unit + ']'
  csv_data = pandas.read_csv(read_file_name, skiprows=skiprows, sep=',', usecols=[name_x, name_y, name_z])
  vector = np.array([csv_data[name_x].to_numpy(), 
                     csv_data[name_y].to_numpy(),
                     csv_data[name_z].to_numpy()])
  return vector

def read_scalar_from_csv(read_file_name, header_name, skiprows=[1,1]):
  csv_data = pandas.read_csv(read_file_name, skiprows=skiprows, sep=',', usecols=[header_name])
  vector = np.array([csv_data[header_name].to_numpy()])
  return vector
