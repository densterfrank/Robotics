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
import math

TARGET_HAZARD_COUNT = 5
globals()['started'] = False
globals()['hazards'] = {}
globals()['frontiers'] = None

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
        frontier = chooseFrontier()
        if not frontier is None:
            moveToMapPose(frontier, True)
        else:
            print("Could not find a frontier")
            break
    print("Finished")
    returnHome()
    stop()

def start():
    print("Starting ...")
    globals()['started'] = True
    globals()['readySub'].unregister()
    print("Started")
    act()

def stop():
    print("Shutting down ...")
    globals()['hazardsSub'].unregister()
    globals()['pathSub'].unregister()
    rospy.sleep(5)
    rospy.signal_shutdown("Now home")

def hazardCount():
    return len(globals()['hazards'])

def chooseFrontier():
    map = globals()['occupancyGrid']
    costmap = globals()['costmap']
    if (map):
        data = map.data
        info = map.info
        width = info.width
        robotPose = globals()['odometry'].pose.pose
        robotX = math.trunc((robotPose.position.x - (info.origin.position.x)) / info.resolution)
        robotY = math.trunc((robotPose.position.y - (info.origin.position.y)) / info.resolution)
        for index, point in enumerate(data):
            frontierX = math.trunc(index / width)
            frontierY = math.trunc(index % width)
            cost = costmap.data[index]
            if (point == -1 and cost < 50): # and isFrontierReachable([robotX, robotY], [frontierX, frontierY])):
                frontier = Pose()
                frontier.position.x = info.origin.position.x + (frontierX + 0.5) * info.resolution
                frontier.position.y = info.origin.position.y + (frontierY + 0.5) * info.resolution
                frontier.position.z = 0
                frontier.orientation.x = 0
                frontier.orientation.y = 0
                frontier.orientation.z = 0
                frontier.orientation.w = 1
                print("Found frontier " + str(frontier.position) + " (" + str(cost) + ")")
                return frontier
    return None

def place(marker: Marker):
    print("Placing " + str(marker.id))
    globals()['hazards'][marker.id] = marker

def move_done_cb(status, result):
    print(status)
    print(result)
    #if result.outcome == MoveBaseResult.SUCCESS:
    #     rospy.loginfo("Follow path action succeeded")
    #else:
    #    rospy.logerr("Follow path action failed: %s", result.message)

def moveToMapPose(pose: Pose, wait: Bool = False, timeout: int = 10):
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
        wait_for_result = moveBaseClient.wait_for_result(timeout = rospy.Duration(timeout))
        # if not wait_for_result:
        #    rospy.logerr("Could not wait for result - action server not available!")
        #    rospy.signal_shutdown("Could not wait for result - action server not available!")
        #else:
        result = moveBaseClient.get_result()
        # print("Result = " + result.result)
        print("Move completed")

def returnHome():
    print("Returning home ...")
    frontier = Pose()
    frontier.position.x = 0
    frontier.position.y = 0
    frontier.position.z = 0
    frontier.orientation.x = 0
    frontier.orientation.y = 0
    frontier.orientation.z = 0
    frontier.orientation.w = 1
    # home = globals()['path'].poses[0].pose
    moveToMapPose(frontier, True, 0)

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
    globals()['occupancyGrid'] = map

def costmapCallback(map: OccupancyGrid):
    globals()['costmap'] = map

def odomCallback(odometry: Odometry):
    globals()['odometry'] = odometry

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
        moveBaseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        globals()['moveBaseClient'] = moveBaseClient

        print("Waiting for move_base server ...")
        wait = moveBaseClient.wait_for_server()
        if not wait:
            rospy.logerr("Could not wait for server - action server not available!")
            rospy.signal_shutdown("Could not wait for server - action server not available!")
            return
        print("Waiting to start ...")
        while (not rospy.is_shutdown()):
            rospy.spin()
        print("Shut down")

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    initialise()