import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
def read_cspace_data():
    name = "test.txt"
    with open(name, "r") as f:
        polygons = []
        polygon = []
        point = np.zeros((1, 2))
        lines = f.readlines()
        i = 0
        nlines = len(lines)
        n_plane = 0
        while i != nlines:
            n_points= 0
            if "Plane" in lines[i]:
                line = lines[i].strip("\n")
                line = line.split(" ")
                n_points = int(line[-1])
                i = i +1
            for j in range(0, n_points):
                line = lines[i+j].strip("\n")
                coord = [float(c) for c in line.split(" ")]
                coord.extend([n_plane])
                polygon.append(np.array(coord))
            i = i + j+1
            polygons.append(polygon)
            polygon = []
            n_plane += 1
    return polygons

ps = read_cspace_data()
for index, p in enumerate(ps):
    ps[index] = np.vstack(p)


fig = plt.figure()
ax = Axes3D(fig)
for i in range(len(ps)):
    x = ps[i][:, 0]
    y = ps[i][:, 1]

    z = ps[i][:, 2]
    verts = [list(zip(x, y, z))]
    collection = Poly3DCollection(verts, alpha=0.9)
    collection.set_facecolor('r')
    collection.set_edgecolor('k')
    ax.add_collection3d(collection)
    ax.set_xlim3d(-10, 10)
    ax.set_ylim3d(-10, 10)
    ax.set_zlim3d(0, 361)
plt.show()
