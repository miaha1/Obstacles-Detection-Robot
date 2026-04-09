#min
import math
import random
import cozmo
import asyncio

from cmap import *
from gui import *
from utils import *
from time import sleep

MAX_NODES = 20000

################################################################################
# NOTE:
# Before you start, please familiarize yourself with class Node in utils.py
# In this project, all nodes are Node object, each of which has its own
# coordinate and parent if necessary. You could access its coordinate by node.x
# or node[0] for the x coordinate, and node.y or node[1] for the y coordinate
################################################################################

def step_from_to(node0, node1, limit=75):
    ############################################################################
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit.
    
    distance = get_dist(node0, node1)
    # if distance btw node0 and node1 is less than limit, return node1
    if distance < limit:
        return node1
    
    angle = np.arctan2(node1.y - node0.y, node1.x - node0.x)
        
    return Node((node0.x + limit * math.cos(angle), node0.y + limit * math.sin(angle)))

    #return node1
    ############################################################################


def node_generator(cmap):
    rand_node = None
    ############################################################################
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Return a Node object
    
    # Loop to find a valid coordinate
    while True:
        # pick a random x & y within the map boundary
        x = random.uniform(0, cmap.width)
        y = random.uniform(0, cmap.height)
        rand_node = Node((x, y))
        
        # check if the node is valid and not inside any obstacle
        if cmap.is_inbound(rand_node) and not cmap.is_inside_obstacles(rand_node):
            return rand_node
    ############################################################################


def RRT(cmap, start):
    cmap.add_node(start)

    map_width, map_height = cmap.get_size()

    # expand tree until we find a solution or reach the maximum number of nodes
    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        # 1. Use CozMap.get_random_valid_node() to get a random node. This
        #    function will internally call the node_generator above
        # 2. Get the nearest node to the random node from RRT
        # 3. Limit the distance RRT can move
        # 4. Add one path from nearest node to random node
        
        # get a random node
        rand_node = cmap.get_random_valid_node()
        
        # get the nearest node to the random node from RRT
        nodes = cmap.get_nodes()
        
        # compare distance between all existing nodes and the random node, and find the nearest one
        nearest_node = min(nodes, key=lambda n: get_dist(n, rand_node))
        
        # step towards the random node from the nearest node
        rand_node = step_from_to(nearest_node, rand_node)
        
        ########################################################################
        sleep(0.01)
        cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            break

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
    else:
        print("Please try again :-(")



async def CozmoPlanning(robot: cozmo.robot.Robot):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent

    ########################################################################
    # Description of function provided in instructions
    # Level cozmo head 
    await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
    # identify target cube and set it as goal
    
    # dynamic offset 
    offset_x = 100
    offset_y = 100
    
    target_cube_id = cozmo.objects.LightCube1Id
    obstacle_cube = [cozmo.objects.LightCube2Id, cozmo.objects.LightCube3Id]
    seen_obstacles = set() # to keep track of seen obstacles
    


    def update_obstacles():

        """Helper function to check for new obstacle cubes and add them to the RRT map."""
        #global cmap
        added_obstacles = False
        for cube_id in obstacle_cube:
            if cube_id not in seen_obstacles:
                cube = robot.world.get_light_cube(cube_id)
                # if cube exist and visible then add it as an obstacle
                if cube is not None and cube.is_visible:
                    seen_obstacles.add(cube_id)
                    cx = cube.pose.position.x + offset_x
                    cy = cube.pose.position.y + offset_y
                    
                    size = 50
                    corners = [
                        Node((cx - size, cy - size)),
                        Node((cx - size, cy + size)),
                        Node((cx + size, cy + size)),
                        Node((cx + size, cy - size))
                    ]
                    cmap.add_obstacle(corners) #update the map with new obstacles
                    added_obstacles = True
        return added_obstacles
    
    
    # Loop to explore if target cube is not visible or if the 
    # path to the target cube is blocked by new obstacles
    while not stopevent.is_set():
        update_obstacles() # check for new obstacles and update the map
        target_cube = robot.world.get_light_cube(target_cube_id)
        if target_cube is not None and target_cube.is_visible:
            break
        
        # Calculate the distance to the center of the arena (cmap.width / 2, cmap.height / 2)
        cx, cy = cmap.width / 2, cmap.height / 2
        dx = cx - (robot.pose.position.x + offset_x)
        dy = cy - (robot.pose.position.y + offset_y)

        # if further than 100mm from the center, move towards the center
        if math.hypot(dx, dy) > 100:
            target_angle = math.atan2(dy, dx)
            # Calculate the difference between where we are facing and where we need to face
            angle_diff = target_angle - robot.pose.rotation.angle_z.radians
            
            #normalize the angle difference to be between -pi and pi
            while angle_diff > math.pi: angle_diff -= 2 * math.pi
            while angle_diff < -math.pi: angle_diff += 2 * math.pi
            
            # Turn and drive a short distance
            await robot.turn_in_place(cozmo.util.radians(angle_diff)).wait_for_completed()
            await robot.drive_straight(cozmo.util.distance_mm(50), cozmo.util.speed_mmps(50)).wait_for_completed()
        else:
            # spin around to scan the room if not visible
            await robot.turn_in_place(cozmo.util.degrees(30)).wait_for_completed()
    
    close_enough = False 
    while not stopevent.is_set() and not close_enough:
        curr_x = robot.pose.position.x + offset_x
        curr_y = robot.pose.position.y + offset_y
        start_node = Node((curr_x, curr_y))
        cmap.set_start(start_node)
        
        # Set the goal to the cube center
        gx = target_cube.pose.position.x + offset_x 
        gy = target_cube.pose.position.y + offset_y
        goal_node = Node((gx, gy))

        #If already within 88mm, face goal and end.
        if math.hypot(gx - curr_x, gy - curr_y) < 88:
            target_angle = math.atan2(gy - curr_y, gx - curr_x)
            angle_diff = target_angle - robot.pose.rotation.angle_z.radians
            while angle_diff > math.pi: angle_diff -= 2 * math.pi
            while angle_diff < -math.pi: angle_diff += 2 * math.pi
            await robot.turn_in_place(cozmo.util.radians(angle_diff)).wait_for_completed()
            close_enough = True
            break
        
        cmap.clear_goals()
        cmap.add_goal(goal_node)
        cmap.reset() # reset the RRT map to find a new path with updated obstacles
        
        print(f"Planning path from ({curr_x:.1f}, {curr_y:.1f}) to ({gx:.1f}, {gy:.1f})")
        RRT(cmap, start_node)
        
        if not cmap.is_solved():
            print("No valid path found, replanting...")
            await asyncio.sleep(1) # wait a bit before replanting
            continue
        
        path = []
        for goal in cmap.get_goals():
            if goal.parent is not None:
                cur = goal
                path.append(cur)
                while cur.parent is not None:
                    path.append(cur.parent)
                    cur = cur.parent
                break
        path.reverse() # reverse the path to get the correct order 
    
    # follow the path and stop in front of the cube found by RRT
        replan_needed = False
        for node in path:
            if stopevent.is_set():
                break

            cx = robot.pose.position.x + offset_x
            cy = robot.pose.position.y + offset_y

            #If we got close enough to the goal cube, stop.
            if math.hypot(gx - cx, gy - cy) < 88:
                print("Reached 88mm threshold during navigation. Facing cube...")
                target_angle = math.atan2(gy - cy, gx - cx)
                angle_diff = target_angle - robot.pose.rotation.angle_z.radians
                while angle_diff > math.pi: angle_diff -= 2 * math.pi
                while angle_diff < -math.pi: angle_diff += 2 * math.pi
                await robot.turn_in_place(cozmo.util.radians(angle_diff)).wait_for_completed()
                close_enough = True
                break
        
            # calculate the angle to the next node in the path
            dx = node.x - (robot.pose.position.x + offset_x)
            dy = node.y - (robot.pose.position.y + offset_y)
            target_angle = math.atan2(dy, dx)
        
            # Calculate the difference between where we are facing and where we need to face
            angle_diff = target_angle - robot.pose.rotation.angle_z.radians
        
            #normalize the angle difference to be between -pi and pi
            while angle_diff > math.pi: angle_diff -= 2 * math.pi
            while angle_diff < -math.pi: angle_diff += 2 * math.pi
        
            # Turn and drive towards the next node in the path
            await robot.turn_in_place(cozmo.util.radians(angle_diff)).wait_for_completed()
            distance = math.hypot(dx, dy)
            await robot.drive_straight(cozmo.util.distance_mm(distance*0.45), cozmo.util.speed_mmps(25)).wait_for_completed()
        
            # check for new obstacles and replan if necessary
            if update_obstacles():
                print("New obstacle detected, replanning...")
                replan_needed = True # Flag that we need to calculate a brand new path
                break # Break out of the path-following loop to trigger the replanning
        
        # reached the goal, exit the planning while-loop
        if close_enough:
            break

        # replanting to avoid obstacles if the path is blocked by new obstacles
        if not replan_needed:
            cx, cy = robot.pose.position.x + offset_x, robot.pose.position.y + offset_y
            if math.hypot(gx - cx, gy - cy) < 88:
                close_enough = True
                print("Successfully navigated to the target cube face!")
            else:
                print("Path ended > 88mm from goal. Re-planning...")

    print("CozmoPlanning Finished.")

################################################################################
#                     DO NOT MODIFY CODE BELOW                                 #
################################################################################

class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            sleep(100)
            cmap.reset()
        stopevent.set()


if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    cmap = CozMap("maps/emptygrid.json", node_generator)
    robot_thread = RobotThread()
    robot_thread.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()
