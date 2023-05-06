import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np

b = bagreader('/home/benny/catkin_ws/src/EECE5554/Project/backward.bag')

LocB = b.message_by_topic('/imu')