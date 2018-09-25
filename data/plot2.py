import plotly as py
from plotly.graph_objs import *
import pandas as pd
import math

my_cols = ['errorid', 'px_est', 'py_est', 'px_meas', 'py_meas', 'vx_est', 'vy_est', 'theta']
with open('output.txt') as f:
    table_error_output = pd.read_table(f, sep=' ', header=None, names=my_cols, lineterminator='\n')

import plotly.offline as py
import matplotlib.pyplot as plt  
from plotly.graph_objs import *

plt.subplot(411)  
plt.xlabel('py_est')
plt.ylabel('px_est')
plt.plot(table_error_output['py_est'],table_error_output['px_est'],marker="+")

plt.subplot(412)  
plt.xlabel('py_meas')
plt.ylabel('px_meas')
plt.plot(table_error_output['py_meas'],table_error_output['px_meas'],marker="*")

plt.subplot(425)  
plt.xlabel('errorid')
plt.ylabel('vx_est')
plt.plot(table_error_output['errorid'],table_error_output['vx_est'],marker="+")
plt.subplot(426)  
plt.xlabel('errorid')
plt.ylabel('vy_est')
plt.plot(table_error_output['errorid'],table_error_output['vy_est'],marker="+")

plt.subplot(414)  
plt.xlabel('errorid')
plt.ylabel('theta')
plt.plot(table_error_output['errorid'],table_error_output['theta'],marker="*")




plt.show()


