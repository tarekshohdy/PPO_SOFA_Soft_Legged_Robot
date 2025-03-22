import numpy as np
import pandas as pd
from numpy import sin, cos, tan, radians, sqrt, arccos, arctan2, rad2deg

# load csv file
raw_data = pd.read_csv('real_leg_rpy.csv', index_col=False)
data = raw_data.copy()

# convert angles to radians
data['roll'] = data['roll'] - data['roll'].iloc[0]
data['pitch'] = data['pitch'] - data['pitch'].iloc[0]
data['yaw'] = data['yaw'] - data['yaw'].iloc[0]
data['L1'] = data['L1'] - data['L1'].iloc[0]
data['L2'] = data['L2'] - data['L2'].iloc[0]
data['L3'] = data['L3'] - data['L3'].iloc[0]

data.drop(['theta1', 'theta2', 'theta3'], axis=1, inplace=True)

print(raw_data.head())
# print(data.head())

L1 = 175 - data['L1'].to_numpy()
L2 = 175 - data['L2'].to_numpy()
L3 = 175 - data['L3'].to_numpy()

roll = data['roll'].to_numpy()
pitch = data['pitch'].to_numpy()
yaw = data['yaw'].to_numpy()

pitch = -pitch
yaw = -yaw

A_3_3 = cos(radians(roll)) * cos(radians(pitch))
A_3_2 = sin(radians(roll)) * cos(radians(pitch))
A_3_1 = -sin(radians(pitch))

s = (L1 + L2 + L3) / 3
k = arccos(A_3_3) / s
phi = arctan2(A_3_2, A_3_1)

x = cos(phi) * (cos(k*s) - 1) / k
y = sin(phi) * (cos(k*s) - 1) / k
z = sin(k*s) / k

# add x y z to data
data['x'] = x
data['y'] = y
data['z'] = z

data['x'].fillna(0, inplace=True)
data['y'].fillna(0, inplace=True)
data['z'].fillna(175, inplace=True)

data.drop(['roll', 'pitch', 'yaw'], axis=1, inplace=True)

data.to_csv('real_leg_xyz.csv', index=False)

print(data.head())