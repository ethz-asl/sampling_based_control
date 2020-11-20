# import numpy as np
# import csv
#
# # csv file name
# filename = "/home/giuseppe/data.txt"
#
# # initializing the titles and rows list
# fields = []
# rows = []
#
# # reading csv file
# with open(filename, 'r') as csvfile:
#     # creating a csv reader object
#     csvreader = csv.reader(csvfile)
#
#     # extracting field names through first row
#     fields = next(csvreader)
#
#     # extracting each data row one by one
#     for row in csvreader:
#         rows.append(row)
#
#         # get total number of rows
#     print("Total no. of rows: %d"%(csvreader.line_num))
#
# # printing the field names
# print('Field names are:' + '\n'.join(field for field in fields))

import numpy as np
from scipy.signal import savgol_filter

import pandas as pd
import matplotlib.pyplot as plt

fig, ax = plt.subplots(3, 3)

df = pd.read_csv("/home/giuseppe/test_input.txt")

i = 0
j = 0
time, data = [], []
for field in df.columns:
    if field.find("/input/recceipt_time") >= 0:
        time = df[field]
        ax[i][j].set_xlabel(field)
    else:
        data = df[field]
        data_filtered = savgol_filter(data, 21, 3)
        ax[i][j].plot(time, data, label="data")
        ax[i][j].plot(time, data_filtered, label="savgol filter data")
        ax[i][j].set_ylabel(field)
        ax[i][j].legend()

        i += 1
        if i == 3:
            i = 0
            j += 1

plt.show()
