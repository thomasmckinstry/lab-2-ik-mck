import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

my_chain  = ikpy.chain.Chain.from_urdf_file("./ur5/ur5_gripper.urdf")

num_samples = 128 
r = 2
u = np.random.uniform(low=-r, high=r, size=(num_samples))
theta = np.random.uniform(low=0, high=2*np.pi, size=(num_samples))

end_points = np.empty((num_samples, 3))

for i in range(num_samples):
    k = u[i]
    j = theta[i]

    x = np.sqrt(np.square(r)-np.square(k)) * np.cos(j)
    y = np.sqrt(np.square(r)-np.square(k)) * np.sin(j)
    z = k
    target = [x, y, z]

    end_pos = my_chain.forward_kinematics(my_chain.inverse_kinematics(target))
    end_points[i] = end_pos[:3,3]

hull = ConvexHull(end_points)
x = []
y = []
z = []

for vertex in hull.vertices:
    x.append(end_points[vertex, 0])
    y.append(end_points[vertex, 1])
    z.append(end_points[vertex, 2])

fig, ax = plt.subplots(subplot_kw={'projection':'3d'})
for simplex in hull.simplices:
    plt.plot(end_points[simplex, 0], end_points[simplex, 1], end_points[simplex, 2], 'k-')
plt.show()
