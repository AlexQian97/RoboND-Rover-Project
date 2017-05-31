import numpy as np
import random

def calculate_dist(pt1, pt2):
    return ((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)**0.5

def line_intersection(node1, node2, rock):
    line1 = [node1, node2]
    #print(line1)
    slope = (node2[1] - node1[1]) / (node2[0] - node1[0])
    slope = -1 / slope
    line2 = [rock, (rock[0] + 1, rock[1] + slope)]
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return (x, y)

def reach_goal(dist, goal, samples, rock_picked):
    if goal in zip(samples[0], samples[1]):
        if goal in rock_picked:
            return True
        else:
            return False
    else:
        if dist < 1:
            return True
        else:
            return False

def mission_planner(nodes, end_nodes, samples):
    original_nodes = nodes[:]
    for sample in zip(samples[0], samples[1]):
        # print(nodes)
        sample_done = False
        # print("sample is")
        # print(sample)
        for node in end_nodes:
            if calculate_dist(sample, node) <= 5:
                # print('close to node')
                # print("node is")
                # print(node)
                nodes.insert(nodes.index(node)+1, sample)
                nodes.insert(nodes.index(node)+2, node)
                sample_done = True
                break
        if not sample_done:
            # print("not close")
            total_dist = 100000
            node1 = None
            node2 = None
            #print(nodes, len(nodes))
            for idx in range(len(original_nodes) - 1):
                dist = calculate_dist(original_nodes[idx], sample) + calculate_dist(original_nodes[idx+1], sample)
                #print(dist)
                if dist < total_dist:
                    total_dist = dist
                    node1 = original_nodes[idx]
                    node2 = original_nodes[idx + 1]
            intersection = line_intersection(node1, node2, sample)
            nodes.insert(nodes.index(node1)+1, intersection)
            nodes.insert(nodes.index(node1)+2, sample)
            nodes.insert(nodes.index(node1)+3, intersection)
            #print(nodes.index(node1))
    return nodes

def adjust_angle(dist, nums):
    magrin = 0.1
    for num in nums:
        lower = num - magrin
        upper = num + magrin
        if (lower <= dist) and (upper >= dist):
            return True
    return False


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!


    if Rover.plan == None:
        print("original!!!!!!!!!!!!!!!")
        print(Rover.nodes, len(Rover.nodes))
        Rover.plan = mission_planner(Rover.nodes, Rover.end_nodes, Rover.samples_pos)
        print(Rover.plan, len(Rover.plan))

    # set goal
    if Rover.goal == None:
        Rover.goal = Rover.plan[1]
        Rover.now = Rover.plan[0]
    # check if it has reached goal
    if Rover.goal != None:
        # calculate goal distance
        Rover.goal_dist = calculate_dist(Rover.goal, Rover.pos)
        Rover.steering_angle = np.arctan2(Rover.goal[1] - Rover.pos[1], Rover.goal[0] - Rover.pos[0]) * 180/np.pi
        # reach goal
        if reach_goal(Rover.goal_dist, Rover.goal, Rover.samples_pos, Rover.rock_picked):
            Rover.plan.pop(0)
            Rover.goal = Rover.plan[1]
            Rover.now = Rover.pos
            Rover.mode = 'stop'
    

    if Rover.nav_angles is not None:
        # print(Rover.goal, Rover.goal in Rover.samples_pos)
        # print(Rover.samples_pos)
        # If we're already in "stop" mode then make different decisions
        if Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                print("stopping")
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                print("stoped")
                # Now we're stopped and we have vision data to see if there's a path forward
                Rover.steering_angle = np.arctan2(Rover.goal[1] - Rover.pos[1], Rover.goal[0] - Rover.pos[0]) * 180/np.pi
                if Rover.steering_angle < 0:
                    Rover.steering_angle += 360
                Rover.steer = Rover.steering_angle - Rover.yaw
                print(Rover.steer)
                if (Rover.steer < 1) & (Rover.steer > -1):
                    print("finish steering")
                    if len(Rover.nav_angles) < Rover.go_forward:
                        print("turning")
                        Rover.throttle = 0
                        # Release the brake to allow turning
                        Rover.brake = 0
                        # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                        Rover.steer = 15#random.choice([-15, 15]) # Could be more clever here about which way to turn
                    if Rover.goal in zip(Rover.samples_pos[0], Rover.samples_pos[1]):
                        print("rock ahead")
                        Rover.throttle = Rover.throttle_set
                        # Release the brake
                        Rover.brake = 0
                        # Set steer to mean angle
                        Rover.steer = np.clip(np.mean(Rover.nav_angles) * 180/np.pi, -3, 3)
                        Rover.mode = 'forward'
                    # If we're stopped but see sufficient navigable terrain in front then go!
                    elif len(Rover.nav_angles) >= Rover.go_forward:
                        print("clear view")
                        # Set throttle back to stored value
                        Rover.throttle = Rover.throttle_set
                        # Release the brake
                        Rover.brake = 0
                        # Set steer to mean angle
                        Rover.steer = np.clip(np.mean(Rover.nav_angles) * 180/np.pi, -15, 15)
                        Rover.mode = 'forward'
                else:
                    print("steering")
                    Rover.throttle = 0
                    Rover.brake = 0

        elif Rover.mode == 'forward': 
            print("forward")
            # Check the extent of navigable terrain
            #print(Rover.goal_dist, adjust_angle(Rover.goal_dist, [0.7, 1, 3, 5, 10, 20]))
            if Rover.goal in zip(Rover.samples_pos[0], Rover.samples_pos[1]):
                if adjust_angle(Rover.goal_dist, [0.5, 1, 3, 10, 20]):
                    print("adjust!!!!!!!!!!!!!!!!!!!!")
                    Rover.mode = 'stop'
            else:
                if adjust_angle(Rover.goal_dist, [3, 10]):
                    print("adjust!!!!!!!!!!!!!!!!!!!!")
                    Rover.mode = 'stop'
            if Rover.mode == 'forward':
                if len(Rover.nav_angles) >= Rover.stop_forward:  
                    # If mode is forward, navigable terrain looks good 
                    # and velocity is below max, then throttle 
                    if Rover.vel < Rover.max_vel:
                        # Set throttle value to throttle setting
                        Rover.throttle = Rover.throttle_set
                    else: # Else coast
                        Rover.throttle = 0
                    Rover.brake = 0
                    # Set steering to average angle clipped to the range +/- 15
                    if (Rover.goal in zip(Rover.samples_pos[0], Rover.samples_pos[1])) | (Rover.goal_dist < 5):
                        Rover.steer = 0
                    else:
                        Rover.steer = np.clip(np.mean(Rover.nav_angles) * 180/np.pi, -15, 15)
                # If there's a lack of navigable terrain pixels then go to 'stop' mode
                elif (len(Rover.nav_angles) < Rover.stop_forward) & (Rover.goal not in zip(Rover.samples_pos[0], Rover.samples_pos[1])):
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
                else:
                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0

    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    if Rover.near_sample and Rover.vel <= 0.2 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.rock_picked.append(Rover.goal)

    return Rover