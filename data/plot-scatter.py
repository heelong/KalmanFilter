import plotly as py
from plotly.graph_objs import *
import pandas as pd
import math

#py.offline.init_notebook_mode(connected=True)

my_cols = ['errorid', 'px_est', 'py_est', 'px_meas', 'py_meas', 'vx_est', 'vy_est','v', 'theta']
with open('output.txt') as f:
    table_ekf_output = pd.read_table(f, sep=' ', header=None, names=my_cols, lineterminator='\n')

    # table_ekf_output

import plotly.offline as py
from plotly.graph_objs import *


# estimations
trace1 = Scatter(
   x=table_ekf_output['px_est'],
   y=table_ekf_output['py_est'],
   xaxis='x2',
   yaxis='y2',
   name='KF- Estimate',
   fillcolor = 'rgb(255,0,0)',
   mode='markers'
)

# Measurements
trace2 = Scatter(
    x=table_ekf_output['px_meas'],
    y=table_ekf_output['py_meas'],
    xaxis='x2',
    yaxis='y2',
    name='Measurements',
	fillcolor = 'rgb(255,255,255)',
    mode = 'markers'
)


data = [trace1,trace2]

layout = Layout(
    xaxis2=dict(

        anchor='x2',
        title='px'
    ),
    yaxis2=dict(

        anchor='y2',
        title='py'
    )
)

fig = Figure(data=data, layout=layout)
py.plot(fig, filename='EKF')

