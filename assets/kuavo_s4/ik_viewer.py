# Import the necessary libraries
import numpy as np

from ikpy.chain import Chain
from ikpy.utils import plot


import matplotlib
matplotlib.use('Qt5Agg')  # 或者 'Qt5Agg'


# Robot Import and Setup
kuavo_left_arm_chain = Chain.from_json_file("./urdf/kuavo_s4_left_arm.json")
kuavo_right_arm_chain = Chain.from_json_file("./urdf/kuavo_s4_right_arm.json")


from mpl_toolkits.mplot3d import Axes3D

fig, ax = plot.init_3d_figure()
kuavo_left_arm_chain.plot([0] * (len(kuavo_left_arm_chain)), ax)
kuavo_right_arm_chain.plot([0] * (len(kuavo_right_arm_chain)), ax)

ax.legend()

plot.show_figure()