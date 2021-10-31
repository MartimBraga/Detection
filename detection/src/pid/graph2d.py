import pandas as pd
import matplotlib.pyplot as plt

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

#medida
for rownum in range(sheet.nrows):
    x.append(sheet.cell_value(rownum, 29))
for rownum in range(sheet.nrows):
    y.append(sheet.cell_value(rownum, 30))
for rownum in range(sheet.nrows):
    z.append(sheet.cell_value(rownum, 5))
for rownum in range(sheet.nrows):
    t.append(sheet.cell_value(rownum, 10))
#efectiva
""" 
for rownum in range(sheet.nrows):
    x.append(sheet.cell_value(rownum, 27))
for rownum in range(sheet.nrows):
    y.append(sheet.cell_value(rownum, 28))
for rownum in range(sheet.nrows):
    z.append(sheet.cell_value(rownum, 29))
for rownum in range(sheet.nrows):
    t.append(sheet.cell_value(rownum, 10)) """

plt.figure(figsize=(10,10))
plt.style.use('seaborn')
plt.scatter(x,y,marker="*",s=100,edgecolors="black",c="yellow")
plt.title("Excel sheet to Scatter Plot")
plt.show()






fig, axs = plt.subplots(2, 2, figsize=(5, 5))
axs[0, 0].plot(data[0], data[1])
axs[1, 0].scatter(data[0], data[1])


plt.show()
