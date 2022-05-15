
from matplotlib import projections
import pandas as pd 
import numpy as np
import time
import matplotlib.pyplot as plt
import sys

file = sys.argv[1]
print(file)

df = pd.read_csv(file)
df.columns = ['x', 'y', 'theta']

df.plot.scatter(x='x', y='y')
plt.show()
# ax = plt.axes(projection='3d')
# X = df['x']
# Y = df['Y']