import plotly as py
from plotly.graph_objs import *
import pandas as pd
import math

my_cols = ['errorid', 'px_est', 'py_est', 'px_meas', 'py_meas', 'vx_est', 'vy_est', 'v','theta']
with open('output.txt') as f:
    table_error_output = pd.read_table(f, sep=' ', header=None, names=my_cols, lineterminator='\n')

import plotly.offline as py
import matplotlib.pyplot as plt  
from plotly.graph_objs import *

plt.subplot(311)  
plt.xlabel('py_est')
plt.ylabel('px_est')
plt.plot(table_error_output['py_est'],table_error_output['px_est'],marker="+")
plt.plot(table_error_output['py_meas'],table_error_output['px_meas'],marker="*")

plt.subplot(312)  
plt.xlabel('errorid')
plt.ylabel('v')
plt.plot(table_error_output['errorid'],table_error_output['v'],marker="+")

plt.subplot(313)  
plt.xlabel('errorid')
plt.ylabel('heading')
plt.plot(table_error_output['errorid'],table_error_output['theta'],marker="*")




plt.show()


