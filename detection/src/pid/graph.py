import plotly.graph_objects as go
import numpy as np
import xlrd
""" from mpl_toolkits import mplot3d
#matplotlib inline
import matplotlib.pyplot as plt

fig = plt.figure()
ax = plt.axes(projection='3d') """

file_location = "/home/martim/catkin_ws/src/detection/src/pid/dataX.xls"

workbook = xlrd.open_workbook(file_location)

sheet = workbook.sheet_by_name('data')

x = []
y = []
z = []
t = []
for rownum in range(sheet.nrows):
    x.append(sheet.cell_value(rownum, 3))
for rownum in range(sheet.nrows):
    y.append(sheet.cell_value(rownum, 4))
for rownum in range(sheet.nrows):
    z.append(sheet.cell_value(rownum, 5))
for rownum in range(sheet.nrows):
    t.append(sheet.cell_value(rownum, 10))

""" ax.plot3D(x, y, z, 'gray') """

fig = go.Figure(data=[go.Scatter3d(x=x, y=y, z=z,
                                   mode='markers')])
fig.show()

""" fig = go.Figure(data=[go.Scatter3d(
    x=x,
    y=y,
    z=z,
    mode='markers',
    marker=dict(
        size=12,
        color=z,                # set color to an array/list of desired values
        colorscale='Viridis',   # choose a colorscale
        opacity=0.8
    )
)])

# tight layout
fig.update_layout(margin=dict(l=0, r=0, b=0, t=0))
fig.show() """