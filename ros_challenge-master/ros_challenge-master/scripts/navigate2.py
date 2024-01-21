#! /usr/bin/env python

import rospy
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import tf
import numpy as np
from skimage.measure import find_contours

TARGET_HAZARD_COUNT = 5
globals()['started'] = False
globals()['hazards'] = {}
globals()['frontiers'] = None

# Frontiers START

class UnionFind:
    """Union-find data structure. Items must be hashable."""

    def __init__(self):
        """Create a new empty union-find structure."""
        self.weights = {}
        self.parents = {}

    def __getitem__(self, obj):
        """X[item] will return the token object of the set which contains `item`"""

        # check for previously unknown object
        if obj not in self.parents:
            self.parents[obj] = obj
            self.weights[obj] = 1
            return obj

        # find path of objects leading to the root
        path = [obj]
        root = self.parents[obj]
        while root != path[-1]:
            path.append(root)
            root = self.parents[root]

        # compress the path and return
        for ancestor in path:
            self.parents[ancestor] = root
        return root

    def union(self, obj1, obj2):
        """Merges sets containing obj1 and obj2."""
        roots = [self[obj1], self[obj2]]
        heavier = max([(self.weights[r],r) for r in roots])[1]
        for r in roots:
            if r != heavier:
                self.weights[heavier] += self.weights[r]
                self.parents[r] = heavier

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
    U = UnionFind()
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
    cluster_trashhole = 0.02
    candidates = groupTPL(candidates, cluster_trashhole)
    frontiers = []
    for i in range(len(candidates)):
        frontiers.append(np.array(candidates[i]))
    return frontiers

def getGoals():
    print("Calculating frontiers ...")
    # Calculate the frontiers.
    frontiers = getFrontiers()
    if (frontiers is not None):
        # Calculate the centroids; a centroid is the centre of mass of a geometric object of uniform density.
        centroids = computeCentroids(frontiers)
        # Calculate the goals; a goal is an (x, y) coordinate.
        goals = utilityFunction(centroids)
        print("Frontiers calculated.")
        return goals
    print("No frontiers could be calculated.")
    return None

# Frontiers END

# Not in use
def search(grid, init, goal, cost, delta, heuristic):
    """
    :param grid: 2D matrix of the world, with the obstacles marked as '1', rest as '0'
    :param init: list of x and y co-ordinates of the robot's initial position
    :param goal: list of x and y co-ordinates of the robot's intended final position
    :param cost: cost of moving one position in the grid
    :param delta: list of all the possible movements
    :param heuristic: 2D matrix of same size as grid, giving the cost of reaching the goal from each cell
    :return: path: list of the cost of the minimum path, and the goal position
    :return: extended: 2D matrix of same size as grid, for each element, the count when it was expanded or -1 if
             the element was never expanded.
    """
    path = []
    val = 1
    visited = [[0 for _ in range(len(grid[0]))] for _ in range(len(grid))]
    visited[init[0]][init[1]] = 1
    expand = [[-1 for _ in range(len(grid[0]))] for _ in range(len(grid))]
    expand[init[0]][init[1]] = 0
    x = init[0]
    y = init[1]
    g = 0
    f = g + heuristic[x][y]
    minList = [f, g, x, y]
    while minList[2:] != goal:
        for i in range(len(delta)):
            x2 = x + delta[i][0]
            y2 = y + delta[i][1]
            if 0 <= x2 < len(grid) and 0 <= y2 < len(grid[0]):
                if visited[x2][y2] == 0 and grid[x2][y2] == 0:
                    g2 = g + cost
                    f2 = g2 + heuristic[x2][y2]
                    path.append([f2, g2, x2, y2])
                    visited[x2][y2] = 1
        if not path:
            return 'fail', expand
        del minList[:]
        minList = min(path)
        path.remove(minList)
        x = minList[2]
        y = minList[3]
        g = minList[1]
        expand[x][y] = val
        val += 1
    return minList, expand

def isFrontierReachable(init, goal):
    map = globals()['occupancyGrid']
    if (map):
        grid = np.asarray(map.data, dtype = np.int8).reshape(map.info.height, map.info.width)
        grid[grid == -1] = 0
        grid[grid > 0] = 1
        heuristic = globals()['costmap']
        cost = 1
        delta = [[-1, 0],  # go up
                [0, -1],  # go left
                [1, 0],  # go down
                [0, 1]]  # go right
        delta_name = ['^', '<', 'v', '>']
        path, expand = search(grid, init, goal, cost, delta, heuristic)
        return path != 'fail'

def act():
    while not hasFinished():
        print("Exploring ...")
        goals = getGoals()
        if (goals is not None and len(goals) > 0):
            quat = tf.transformations.quaternion_from_euler(0, 0, 0)
            print("Moving to " + str(len(goals)) + " waypoints.")
            for (index, goal) in enumerate(goals):
                waypoint = createPose([goal[1], goal[0]], quat)
                publishWaypoint(waypoint, index)
            for goal in goals:
                waypoint = createPose([goal[1], goal[0]], quat)
                moveToMapPose(waypoint)
        else:
            print("Could not find frontiers")
            break
    print("Finished")
    returnHome()
    stop()

def start():
    print("Starting ...")
    globals()['started'] = True
    globals()['readySub'].unregister()
    current_position, current_quaternion = get_current_pose('/map', '/odom')
    # quat = tf.transformations.quaternion_from_euler(0, 0, 0)
    home = createPose(current_position, current_quaternion)
    print("Home = " + str(home.position))
    globals()['home'] = home
    print("Started")
    act()

def createPose(position, quaternion):
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = 0
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose

def stop():
    print("Shutting down ...")
    globals()['hazardsSub'].unregister()
    globals()['pathSub'].unregister()
    rospy.sleep(5)
    rospy.signal_shutdown("Now home")

def hazardCount():
    return len(globals()['hazards'])

def place(marker: Marker):
    print("Placing " + str(marker.id))
    globals()['hazards'][marker.id] = marker

def move_done_cb(status, result):
    print(status)
    print(result)

def moveToMapPose(pose: Pose, wait: Bool = True, timeout: int = 60):
    print('Moving to ' + str(pose.position))
    targetPose = PoseStamped()
    targetPose.header.frame_id = "map"
    targetPose.header.stamp = rospy.Time.now()
    targetPose.pose = pose
    goal = MoveBaseGoal()
    goal.target_pose = targetPose
    moveBaseClient = globals()['moveBaseClient']
    moveBaseClient.send_goal(goal, done_cb = move_done_cb)
    if wait:
        success = moveBaseClient.wait_for_result(timeout = rospy.Duration(timeout))
        if not success:
            moveBaseClient.cancel_goal()
        state = moveBaseClient.get_state()
        print("Move state = " + str(state))

def returnHome():
    print("Returning home ...")
    home = globals()['home']
    moveToMapPose(home, True, 0)

def hasFinished():
    return globals()['hazards'] and len(globals()['hazards']) >= TARGET_HAZARD_COUNT

def setPath(path: Path):
    globals()['path'] = path

def setFrontiers(frontiers: MarkerArray):
    globals()['frontiers'] = frontiers

def ready(ready: Bool):
    print('Ready')
    start()

def cancelGoals(msg: Bool):
    moveBaseClient = globals()['moveBaseClient']
    moveBaseClient.cancel_all_goals()

def mapCallback(map: OccupancyGrid):
    info = map.info
    data = map.data
    mapRaw = np.array(data)
    mapRaw = mapRaw.reshape(info.height, info.width)
    globals()['occupancyGrid'] = mapRaw
    globals()['occupancyGridOrigin'] = np.array([info.origin.position.x, info.origin.position.y, info.origin.position.z])
    globals()['occupancyGridResolution'] = info.resolution
    rospy.sleep(1)

def costmapCallback(map: OccupancyGrid):
    globals()['costmap'] = map

def odomCallback(odometry: Odometry):
    globals()['odometry'] = odometry

def get_current_pose(target_frame, source_frame):
    # rospy.Time(0) (in the context of tf) returns the latest available transform
    position, quaternion = globals()['listener'].lookupTransform(target_frame, source_frame, rospy.Time(0))
    return position, quaternion

def publishWaypoint(waypoint: Pose, id):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time().now()
    marker.pose = waypoint
    marker.id = id
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1
    globals()['waypointsPub'].publish(marker)

def initialise():
    try:
        print("Initialising navigation ...")
        rospy.init_node('navigate', anonymous=False, disable_signals=True)
        globals()['readySub'] = rospy.Subscriber('/challenge/ready', Bool, ready)
        globals()['hazardsSub'] = rospy.Subscriber('/hazards', Marker, place)
        globals()['pathSub'] = rospy.Subscriber('/path', Path, setPath)
        globals()['cancelGoalsSub'] = rospy.Subscriber('/challenge/cancelGoals', Bool, cancelGoals)
        globals()['goalPub'] = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        globals()['frontiersSub'] = rospy.Subscriber('/explore/frontiers', MarkerArray, setFrontiers)
        globals()['mapSub'] = rospy.Subscriber('/map', OccupancyGrid, mapCallback)
        globals()['costmapSub'] = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, costmapCallback)
        globals()['odomSub'] = rospy.Subscriber('/odom', Odometry, odomCallback)
        globals()['waypointsPub'] = rospy.Publisher('/waypoints', Marker, queue_size=100)
        moveBaseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        globals()['moveBaseClient'] = moveBaseClient
        globals()['listener'] = tf.TransformListener()
        print("Waiting for move_base server ...")
        wait = moveBaseClient.wait_for_server()
        if not wait:
            rospy.logerr("Could not wait for server - action server not available!")
            rospy.signal_shutdown("Could not wait for server - action server not available!")
            return
        print("Waiting for occupancy grid ...")
        rospy.wait_for_message("/map", OccupancyGrid)
        print("Ready to start ...")
        while (not rospy.is_shutdown()):
            rospy.spin()
        print("Shut down")

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    initialise()