import rospkg
from os.path import join
import numpy as np
from run import path_coord_to_gazebo_coord, compute_distance, INIT_POSITION, GOAL_POSITION

if __name__ == "__main__":
    rospack = rospkg.RosPack()
    base_path = rospack.get_path('jackal_helper')
    path_file_name = join(base_path, "worlds/BARN/path_files", "path_100.npy")
    path_array = np.load(path_file_name)
    # print(path_array)
    path_array = [path_coord_to_gazebo_coord(*p) for p in path_array]
    # print(path_array)
    path_array = np.insert(path_array, 0, (INIT_POSITION[0], INIT_POSITION[1]), axis=0)
    path_array = np.insert(path_array, len(path_array), (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1]), axis=0)
    # path_length = 0
    print(path_array[:-1])
    print(path_array[1:])
    # for p1, p2 in zip(path_array[:-1], path_array[1:]):
    #     path_length += compute_distance(p1, p2)
    # print(1)