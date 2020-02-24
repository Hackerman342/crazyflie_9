import json
from mapping import Mapping
import matplotlib.pyplot as plt

# Mapping tutorial

# Creating a map object. The main thing with this is that it has a variable called matrix.
# This matrix is the map, where 0 is free space, 1 is wall, 4 is aruco marker and 5 is a road sign
# You can create more interesting maps by creating a new json file with a more interesting layout
# The argument "awesome.world.json" demands that the json file is in the same folder.
mapp = Mapping("awesome.world.json", 0.1, 2)

# This gives us the map matrix, which we can use to do path planning with.
matrx = mapp.matrix

# Since the matrix doesn't have negative indices we need to change the axis when plotting the image
plt.imshow(matrx, extent=[-matrx.shape[1]/2., matrx.shape[1]/2., -matrx.shape[0]/2., matrx.shape[0]/2.])

plt.show()