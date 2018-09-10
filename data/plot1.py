import plotly as py
from plotly.graph_objs import *
import pandas as pd
import math

#py.offline.init_notebook_mode(connected=True)

my_cols = ['errorid', 'px_error', 'py_error', 'vx_error', 'vy_error']
with open('output2.txt') as f:
    table_error_output = pd.read_table(f, sep=' ', header=None, names=my_cols, lineterminator='\n')

    # table_error_output

import plotly.offline as py
import matplotlib.pyplot as plt  
from plotly.graph_objs import *


plt.subplot(221)  
plt.xlabel('index')
plt.ylabel('x error')
plt.scatter(table_error_output['errorid'],table_error_output['px_error'],marker="+")


plt.subplot(222)  
plt.xlabel('index')
plt.ylabel('y error')
plt.scatter(table_error_output['errorid'],table_error_output['py_error'],marker="+")

plt.subplot(223)  
plt.xlabel('index')
plt.ylabel('vx error')
plt.scatter(table_error_output['errorid'],table_error_output['vx_error'],marker="*")

plt.subplot(224)  
plt.xlabel('index')
plt.ylabel('vy error')
plt.scatter(table_error_output['errorid'],table_error_output['vy_error'],marker="*")

plt.show()


