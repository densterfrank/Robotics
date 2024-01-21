import numpy as np
import union_find
from skimage.measure import find_contours

def get_current_pose(target_frame, source_frame):
    # rospy.Time(0) in the context of tf returns the latest available transform
    position, quaternion = transformListener.lookupTransform(target_frame, source_frame, rospy.Time(0))
    return position, quaternion

def utilityFunction(centroids):
    utility_array = np.zeros((centroids.shape[0], centroids.shape[1]))
    utility_array = np.copy(centroids)
    current_position, current_quaternion = get_current_pose('/map', '/odom')

    for index in range(len(centroids)):
        man_dist = abs(current_position[0] - centroids[index][0]) + abs(current_position[1] - centroids[index][1])
        utility = centroids[index][2] / man_dist
        utility_array[index][2] = utility
    index = np.argsort(utility_array[:, 2])
    utility_array[:] = utility_array[index]
    utility_array = utility_array[::-1]
    goals = []
    for i in range(3):
        coordinate = []
        if i < len(utility_array):
            coordinate = [utility_array[i][0], utility_array[i][1]]
            goals.append(coordinate)
    return np.array(goals)

def computeCentroids(list_of_arrays):
    resolution = globals()['occupancyGridResolution']
    centroids = []
    for index in range(len(list_of_arrays)):
        length = list_of_arrays[index].shape[0]
        real_length = np.round(length * resolution * 100)
        sum_x = np.sum(list_of_arrays[index][:, 0])
        x = np.round(sum_x / length, 2)
        sum_y = np.sum(list_of_arrays[index][:, 1])
        y = np.round(sum_y / length, 2)
        centroids.append([x, y, real_length])
    centroids = np.array(centroids)
    return centroids

def groupTPL(TPL, distance=1):
    U = union_find.UnionFind()
    for (i, x) in enumerate(TPL):
        for j in range(i + 1, len(TPL)):
            y = TPL[j]
            if max(abs(x[0] - y[0]), abs(x[1] - y[1])) <= distance:
                U.union(x, y)
    disjSets = {}
    for x in TPL:
        s = disjSets.get(U[x], set())
        s.add(x)
        disjSets[U[x]] = s
    return [list(x) for x in disjSets.values()]

def getFrontiers():
    rawMap = globals()['occupancyGrid']
    origin = globals()['occupancyGridOrigin']
    resolution = globals()['occupancyGridResolution']
    savedMap = np.copy(rawMap)
    contours_negative = find_contours(savedMap, -1.0, fully_connected='high')
    contours_positive = find_contours(savedMap,  1.0, fully_connected='high')
    if len(contours_negative) == 0 or len(contours_positive) == 0:
        return None
    contours_negative = np.concatenate(contours_negative, axis=0)
    for index in range(len(contours_negative)):
        contours_negative[index][0] = round(contours_negative[index][0] * resolution + origin[0], 2)
        contours_negative[index][1] = round(contours_negative[index][1] * resolution + origin[1], 2)
    contours_positive = np.concatenate(contours_positive, axis=0)
    for index in range(len(contours_positive)):
        contours_positive[index][0] = round(contours_positive[index][0] * resolution + origin[0], 2)
        contours_positive[index][1] = round(contours_positive[index][1] * resolution + origin[1], 2)
    set_negative = set([tuple(x) for x in contours_negative])
    set_positive = set([tuple(x) for x in contours_positive])
    candidates = set_negative.difference(set_positive)
    candidates = [x for x in candidates]
    cluster_trashhole = 0.1
    candidates = groupTPL(candidates, cluster_trashhole)
    frontiers = []
    for i in range(len(candidates)):
        frontiers.append(np.array(candidates[i]))
    return frontiers

def getGoals():
    frontiers = getFrontiers()
    centroids = computeCentroids(frontiers)
    goals = utilityFunction(centroids)
    return goals
