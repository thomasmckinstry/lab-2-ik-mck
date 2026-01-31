from os.path import join
import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt

def collision_detection(plane_vector, plane_point, target_position):
    my_chain  = ikpy.chain.Chain.from_urdf_file("./ur5/ur5_gripper.urdf")

    fig, ax = plot_utils.init_3d_figure()
    ax.set_box_aspect((1, 1, 1))

    real_frame = my_chain.forward_kinematics(my_chain.inverse_kinematics(target_position), full_kinematics=True)
    joint_positions = [x[:3, 3] for x in real_frame]

    my_chain.plot(my_chain.inverse_kinematics(target_position), ax)

    axes = [0, 1, 2]

    plane_vector = plane_vector / np.linalg.norm(plane_vector) # Get unit vector

    main_axis = np.argmax(plane_vector)
    axes = list(filter(lambda x: x != main_axis, axes))

    a = np.linspace(-1, 1, 10) 
    b = np.linspace(-1, 1, 10) 
    a, b = np.meshgrid(a, b)
    c = (a * (-plane_vector[axes[0]] * plane_vector[main_axis]) + b * (-plane_vector[axes[1]] / plane_vector[main_axis])) + plane_point[main_axis] # Slope along the x axis with a slope of 2

    meshes = [None] * 3
    meshes[axes[0]] = a
    meshes[axes[1]] = b
    meshes[main_axis] = c

    ax.plot_surface(meshes[0], meshes[1], meshes[2], alpha=0.20, shade=False)

    for i in range(len(joint_positions) - 1):
        distance = np.abs(np.subtract(joint_positions[i], joint_positions[i+1]))
        magnitude = np.linalg.norm(distance)
        unit = distance / magnitude

        intersect = np.dot(unit, plane_vector)
        if intersect != 0:
            d = (np.dot((plane_point - joint_positions[i]), plane_vector)) / intersect
            collision = joint_positions[i] + unit * d
            collision_dist_a = np.linalg.norm(np.abs(np.subtract(joint_positions[i], collision)))
            collision_dist_b = np.linalg.norm(np.abs(np.subtract(joint_positions[i + 1], collision)))
            if collision_dist_a <= magnitude and collision_dist_b <= magnitude:
                print("Collision on joints", i, i+1)
                ax.scatter(collision[0], collision[1], collision[2], color='red')
                return True
    plt.show()

collision_detection([0.5,0.5,0.75],[0,0,0.5],[-0.5,0,0.25])
