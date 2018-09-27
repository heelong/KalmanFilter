import plotly as py
from plotly.graph_objs import *
import pandas as pd
import math

#py.offline.init_notebook_mode(connected=True)

my_cols = ['errorid', 'px_error', 'py_error', 'vx_error', 'vy_error','theta']
with open('output2.txt') as f:
    table_error_output = pd.read_table(f, sep=' ', header=None, names=my_cols, lineterminator='\n')

    # table_error_output

import plotly.offline as py
import matplotlib.pyplot as plt  
from plotly.graph_objs import *


plt.subplot(321)  
plt.xlabel('index')
plt.ylabel('x error')
#plt.scatter(table_error_output['errorid'],table_error_output['px_error'],marker="+")
plt.plot(table_error_output['errorid'],table_error_output['px_error'],marker="+")

plt.subplot(322)  
plt.xlabel('index')
plt.ylabel('y error')
#plt.scatter(table_error_output['errorid'],table_error_output['py_error'],marker="+")
plt.plot(table_error_output['errorid'],table_error_output['py_error'],marker="+")

plt.subplot(323)  
plt.xlabel('index')
plt.ylabel('vx error')
#plt.scatter(table_error_output['errorid'],table_error_output['vx_error'],marker="*")
plt.plot(table_error_output['errorid'],table_error_output['vx_error'],marker="*")

plt.subplot(324)  
plt.xlabel('index')
plt.ylabel('vy error')
#plt.scatter(table_error_output['errorid'],table_error_output['vy_error'],marker="*")
plt.plot(table_error_output['errorid'],table_error_output['vy_error'],marker="*")

plt.subplot(313)  
plt.xlabel('index')
plt.ylabel('Heading')
plt.plot(table_error_output['errorid'],table_error_output['theta'],marker="*")

plt.show()


