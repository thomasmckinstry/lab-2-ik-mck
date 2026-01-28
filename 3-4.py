from os.path import join
import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt

my_chain  = ikpy.chain.Chain.from_urdf_file("./ur5/ur5_gripper.urdf")

fig, ax = plot_utils.init_3d_figure()
ax.set_box_aspect((1, 1, 1))

target_position = [0, 1, 0]
real_frame = my_chain.forward_kinematics(my_chain.inverse_kinematics(target_position), full_kinematics=True)
joint_positions = [x[:3, 3] for x in real_frame]

my_chain.plot(my_chain.inverse_kinematics(target_position), ax)

# Horizontal plane along z = 0.5
"""
x = np.linspace(-1, 1, 10) 
y = np.linspace(-1, 1, 10)
x, y = np.meshgrid(x, y)
z = np.ones((10,10)) * 0.5
ax.plot_surface(x, y, z, alpha=0.75)
"""

# Vertical plane along y = 0.5
"""
x = np.linspace(-1, 1, 10)
z = np.linspace(-1, 1, 10)
x, z = np.meshgrid(x, z)
y = np.ones((10,10)) * 0.5 
ax.plot_surface(x, y, z, alpha=0.75)
"""

# Vertical plane along x = 0.25
"""
y = np.linspace(-1, 1, 10)
z = np.linspace(-1, 1, 10)
y, z = np.meshgrid(y, z)
x = np.ones((10,10)) * 0.5 
ax.plot_surface(x, y, z, alpha=0.75)
"""

# For my collision detection I'd like to have variables defining the plane and be able to use those to determine the collision.
# A plane in 3D could be defined by a function with respect to x, y, z, as well as an offset from each one.
# For a slope along the x axis you make the independent value equal to x

# If I have a function to find the coordinate of the plane for a given pair of coordinates (ex. x, y) I can tell what side of the plane a given points is own.
# This approach isn't plug and play though, since if a plane is vertical or horizontal I don't know which points to include.

# See wikipedia page for Line-plane intersection
# I can find a point of intersection for a line defined with the coordinates of the joints and the plane
# If x, y, or z is greater than the intersection point for one joint, and less than for the other joint, then they lie on opposite sides of the plane 
# I need to be able to define the plane with a vector that is normal to it. For the unit vector representing the plane, the largest part of the unit vector will be the axis that the plane is in reference to.
# ex. Unit vector <0, 0, 1> is in reference to the Z axis.
# Make mesh grid out of axes with smaller portions. Make coordinate map for primary axes by x * a + y * b where a and b are the portion of the vector in those directions. Sum point onto the coordinate map for offset.
axes = [0, 1, 2]

plane_vector = np.array([0.5, 2, 0])
plane_vector = plane_vector / np.linalg.norm(plane_vector) # Get unit vector
plane_point = [0, 0.5, 0] #TODO: For now only  works with offset in the main_axis

main_axis = np.argmax(plane_vector)
axes = list(filter(lambda x: x != main_axis, axes))

a = np.linspace(-1, 1, 10) 
b = np.linspace(-1, 1, 10) 
a, b = np.meshgrid(a, b)
#a += plane_point[axes[0]]
c = (a * (-plane_vector[axes[0]] * plane_vector[main_axis]) + b * (-plane_vector[axes[1]] / plane_vector[main_axis])) + plane_point[main_axis] # Slope along the x axis with a slope of 2

meshes = [None] * 3
meshes[axes[0]] = a
meshes[axes[1]] = b
meshes[main_axis] = c

ax.quiver(plane_point[0], plane_point[1], plane_point[2], plane_vector[0], plane_vector[1], plane_vector[2], color="Green")
ax.plot_surface(meshes[0], meshes[1], meshes[2], alpha=0.75)

for i in range(len(joint_positions) - 1):
#    print("A:", joint_positions[i], "B:", joint_positions[i + 1])
    distance = np.abs(np.subtract(joint_positions[i], joint_positions[i+1]))
    magnitude = np.linalg.norm(distance)
#    print("Dist and Magnitude:", distance, magnitude)
    unit = distance / magnitude
    #print(unit)

    intersect = np.dot(unit, plane_vector)
    if intersect != 0:
        d = (np.dot((plane_point - joint_positions[i]), plane_vector)) / intersect
        collision = joint_positions[i] + unit * d
        collision_dist_a = np.linalg.norm(np.abs(np.subtract(joint_positions[i], collision)))
        collision_dist_b = np.linalg.norm(np.abs(np.subtract(joint_positions[i + 1], collision)))
        #print("Collision a", collision_dist_a, "Collision b", collision_dist_b, magnitude)
        if collision_dist_a <= magnitude and collision_dist_b <= magnitude:
            print("Collision on joints", i, i+1)
            ax.scatter(collision[0], collision[1], collision[2], color='red')
            break
#    print()
plt.show()
