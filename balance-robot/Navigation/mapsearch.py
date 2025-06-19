
import numpy as np
import cv2
import math
import random
import os
from math import sin, cos, atan2, pi, sqrt
from collections import deque

# 常量定义
MAP_SIZE = 250
ROBOT_RADIUS = 4
SENSOR_RANGE = 75
SENSOR_ERROR = 1
ROTATION_STEP = math.pi / 36  # 5 deg
MOVE_STEP = 1.0
FRAME_RATE = 30
VIDEO_FILE = 'robot_navigation_func.mp4'

limited_map = None
accumulated_map = None
warning_map = None
stepU = 0
times = 0

def create_robot(start_pos, goal_pos):

    return {
        'pos': [start_pos[0], start_pos[1]],
        'dist': 0.0,
        'heading': 0.0,
        'actual_pos': np.array(start_pos, dtype=float),
        'goal': np.array([np.array(goal_pos, dtype=int)]),
        'path': [np.array(start_pos, dtype=int)],
        'planned_path': np.array([np.array(goal_pos, dtype=int)]),

        'action': 'scan',
        'isReturn': False,
        'histroy_pos': [start_pos[0], start_pos[1]],
        'isOut' : False
    }


# moving -----------------------------------------------------------------

def rotate_step(robot, target_angle):
    #print("angle diff", target_angle, robot['heading'])

    angle_diff = (target_angle - robot['heading']) % (2 * math.pi)
    
    if angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    
    if abs(angle_diff) > ROTATION_STEP / 2:
        robot['heading'] += math.copysign(ROTATION_STEP, angle_diff)
        robot['heading'] %= 2 * math.pi
        return False
        
    else:
        robot['heading'] = target_angle
        return True

def move_step(robot, target_pos, obstacle_map):

    angle = robot['heading']
    real_vec = np.array([cos(angle), sin(angle)])
    target_vec = target_pos - robot['pos']
    target_dist = np.linalg.norm(target_vec)
    
    if target_dist > MOVE_STEP:

        if (np.dot(real_vec, target_vec) < 0):
            robot['isOut'] = True

        move_vec = real_vec * MOVE_STEP
        new_pos = robot['actual_pos'] + move_vec
        
        if not check_collision(new_pos, obstacle_map):
            robot['actual_pos'] = new_pos
            robot['pos'] = [int(robot['actual_pos'][0]), int(robot['actual_pos'][1])]
            robot['path'].append(robot['pos'].copy())
            return False
        else:
            return True
    else:
        robot['pos'] = [int(target_pos[0]), int(target_pos[1])]
        robot['path'].append(robot['pos'].copy())
        robot['planned_path'] = np.delete(robot['planned_path'], 0, 0)
        return True
    
def execute_move(robot, obstacle_map):
    
    target_pos = robot['planned_path'][0]
    #print(robot['planned_path'])
    target_vec = target_pos - robot['pos']
    target_dir = math.atan2(target_vec[1], target_vec[0])

    if robot['action'] == 'rotate':
        done = rotate_step(robot, target_dir)
        #print(done)
        if done:
            robot['action'] = 'move'
    
    if robot['action'] == 'move':
        done = move_step(robot, target_pos, obstacle_map)
        if robot['isReturn']:
            print('robot: ', robot['pos'],', target original point: ', target_pos)
        
        return done


# scanning -----------------------------------------------------------------



def distance_sensor(robot, obstacle_map):

    end_x = robot['pos'][0] + (SENSOR_RANGE + ROBOT_RADIUS) * math.cos(robot['heading'])
    end_y = robot['pos'][1] + (SENSOR_RANGE + ROBOT_RADIUS) * math.sin(robot['heading'])
    
    distance = SENSOR_RANGE
    for r in range(1, SENSOR_RANGE + 1):
        x = int(robot['pos'][0] + (r + ROBOT_RADIUS) * math.cos(robot['heading']))
        y = int(robot['pos'][1] + (r + ROBOT_RADIUS) * math.sin(robot['heading']))
        
        if not (0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE):
            distance = r - 1
            break
            
        if obstacle_map[y, x]:
            distance = r
            break
    
    if (SENSOR_RANGE - SENSOR_ERROR <= distance <=  SENSOR_RANGE + SENSOR_ERROR):
        distance = SENSOR_RANGE*5

    return max(0, distance + random.randint(-SENSOR_ERROR, SENSOR_ERROR))

def update_known_map(robot, known_map, obstacle_map):

    dist = distance_sensor(robot, obstacle_map)

    if (dist > SENSOR_RANGE):
        return known_map
    
    x = int(robot['pos'][0] + dist * math.cos(robot['heading']))
    y = int(robot['pos'][1] + dist * math.sin(robot['heading']))

    if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
        known_map[y, x] = 0
    
    for dx in range(-ROBOT_RADIUS-1, ROBOT_RADIUS+2):
        for dy in range(-ROBOT_RADIUS-1, ROBOT_RADIUS+2):
            x, y = int(robot['pos'][0]) + dx, int(robot['pos'][1]) + dy
            if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
                dist_to_robot = math.sqrt(dx**2 + dy**2)
                if dist_to_robot <= ROBOT_RADIUS + 0.5:
                    known_map[y, x] = 255
    
    return known_map

def check_collision(pos, obstacle_map):

    x, y = int(pos[0]), int(pos[1])
    for dx in range(-ROBOT_RADIUS, ROBOT_RADIUS+1):
        for dy in range(-ROBOT_RADIUS, ROBOT_RADIUS+1):
            px, py = x + dx, y + dy
            if 0 <= px < MAP_SIZE and 0 <= py < MAP_SIZE:
                if math.sqrt(dx**2 + dy**2) <= ROBOT_RADIUS and obstacle_map[py, px]:
                    return True
    return False

def initial_scan(robot, initial_angle, known_map, obstacle_map):
    global stepU
    if robot['action'] == 'scan':
        if robot['heading']-initial_angle < 2 * math.pi - ROTATION_STEP:
            robot['heading'] += ROTATION_STEP
            known_map = update_known_map(robot, known_map, obstacle_map)
            return False, known_map
        else:
            robot['heading'] = initial_angle
            robot['action'] = 'plan'
            stepU = 0
            return True, known_map
    return False, known_map


# pathes finding -----------------------------------------------------------------



def create_limited_area_map(known_map, size):

    known_map = np.uint8(known_map)

    kernel = np.ones((size, size), np.uint8)
    
    inverted_map = 255 - known_map
    limited_area_map = cv2.dilate(inverted_map, kernel, iterations=3)
    limited_area_map = 255 - limited_area_map 

    return limited_area_map



def line_crosses_limited_area(p1, p2, limited_map):
    x1, y1 = p1[0], p1[1]
    x2, y2 = p2[0], p2[1]

    dx = x2 - x1
    dy = y2 - y1
    length = max(abs(dx), abs(dy))
    
    if length == 0:
        return limited_map[int(y1), int(x1)] < 200

    dx /= length
    dy /= length

    x, y = x1, y1
    for _ in range(int(length) + 1):
        intX = int(round(x))
        intY = int(round(y))

        if (0 <= intY < limited_map.shape[0] and 
            0 <= intX < limited_map.shape[1]):
            if limited_map[intY, intX] < 200:
                return True
        
        x += dx
        y += dy
    
    return False

def recursive_expand(accumulated_map, limited_map, point_before, goal_pos, max_depth, current_depth, angle_threshold = 10):

    angle_threshold_rad = angle_threshold * pi / 180

    if not line_crosses_limited_area(point_before, goal_pos, limited_map):
        #print("not cross: ", point_before, goal_pos)
        return (True, None, accumulated_map)
    
    if current_depth > max_depth:
        #accumulated_map[point_before[0], point_before[1]] = current_depth
        return (False, None, accumulated_map)
    
    isFound = False
    next_points = []
    found_path = []

    cv2.imwrite(f"frames/map2.png", limited_map)
    
    for i in range(5, MAP_SIZE-5, 3):
        for j in range(5, MAP_SIZE-5, 3):

            global times
            times += 1

            if i == point_before[0] and j == point_before[1]:
                continue

            if limited_map[i, j] < 200:
                continue

            #print(accumulated_map[i, j], (i, j), current_depth)
            if accumulated_map[i, j] > 0 and current_depth >= accumulated_map[i, j]:
                continue
                
            angle_to_point = atan2(j - point_before[1], i - point_before[0])
            angle_to_goal = atan2(goal_pos[1] - point_before[1], goal_pos[0] - point_before[0])
            
            if abs(angle_to_goal - angle_to_point) < angle_threshold_rad:
                continue

            if line_crosses_limited_area(point_before, [i, j], limited_map):
                continue

            accumulated_map[i, j] = current_depth
            dist = np.linalg.norm(np.array([i, j])-goal_pos)
            if line_crosses_limited_area(point_before, [i, j], warning_map):
                dist += 1000
            next_points.append([dist, [i, j]])


    next_points.sort()
    #print(point_before, len(next_points))

    for point in next_points:
        isFound, partial_path, accumulated_map = recursive_expand(
            accumulated_map, limited_map, 
            point[1], goal_pos, 
            max_depth, current_depth + 1, 
            angle_threshold
        )
        
        if isFound and partial_path:
            found_path = [point[1]] + partial_path
            return (True, found_path, accumulated_map)
        elif isFound:
            return (True, [point[1]], accumulated_map)
    
    return (False, None, accumulated_map)



def bfs_path_planning(robot, known_map):

    global limited_map

    limited_map = create_limited_area_map(known_map, 4)

    global warning_map

    warning_map = create_limited_area_map(known_map, 10)

    if not line_crosses_limited_area(robot['planned_path'][0], robot['pos'], limited_map):
        return

    cv2.imwrite(f"frames/map5.png", warning_map)

    global accumulated_map
    accumulated_map = np.zeros_like(known_map, dtype=np.uint8)
    accumulated_map += 255

    global times
    times = 0

    start_pos = robot['pos']
    goal_pos = robot['goal'][0]
    heading = robot['heading']
    
    ray_points = []
    x, y, dx, dy = start_pos[0], start_pos[1], cos(heading), sin(heading)

    accumulated_map[x, y] = 0

    if not line_crosses_limited_area(start_pos, goal_pos, limited_map):
        #print("not cross: ", start_pos, goal_pos)
        robot['planned_path'] = np.array([goal_pos])
        cv2.imwrite(f"frames/map4.png", np.clip(accumulated_map*30, 0, 255))
        return
    else:
        pass

    x += 2*dx
    y += 2*dy
    
    while 0 <= x < MAP_SIZE-2 and 0 <= y < MAP_SIZE-2:
        x += dx
        y += dy

        intX = int(x)
        intY = int(y)

        if (intX, intY) not in ray_points and (intX != start_pos[0] or intY != start_pos[1]):

            if limited_map[intY, intX] < 200:
                    
                break

            if limited_map[intX, intY] > 200:
                
                if line_crosses_limited_area(start_pos, (intX, intY), limited_map):
                    
                    break
                else:
                    accumulated_map[intX, intY] = 3
                    dist = np.linalg.norm(np.array([intX, intY])-goal_pos)+abs(x-start_pos[0])*3+abs(y-start_pos[1])*3
                    if line_crosses_limited_area(start_pos, (intX, intY), warning_map):
                        #print("+++")
                        dist += 1000
                    ray_points.append([dist, [intX, intY]])

            else:
                break

    ray_points.sort()
    
    
    for i in range(len(ray_points)):
        #print('point: ',ray_points[i][1])
        isFound, found_points, accumulated_map = recursive_expand(accumulated_map, limited_map, ray_points[i][1], goal_pos, 4, 4) 
        cv2.imwrite(f"frames/map4.png", np.clip(accumulated_map*30, 0, 255)) 
        
        if (found_points == None):
            found_points = []
        
        if isFound:
            robot['planned_path'] = np.array([ray_points[i][1]] + found_points + [goal_pos])
            #print(122, ray_points[i][1], robot['planned_path'])
            #print(robot['pos'], robot['planned_path'])
            return
        
    cv2.imwrite(f"frames/map4.png", np.clip(accumulated_map*30, 0, 255))
    
    if not (robot['histroy_pos'][0] == robot['pos'][0] and robot['histroy_pos'][1] == robot['pos'][1]):
        robot['histroy_pos'] = robot['pos'].copy()
        bfs_path_planning(robot, known_map)
    else:
        #print(robot['planned_path'], robot['path'])
        print('return')
        if len(robot['path']) > 0 and not robot['isReturn']:
            robot['goal'] = np.array([robot['path'][0], robot['goal'][0]])
            print('goal: ', robot['goal'])
            robot['isReturn'] = True


# initialise -----------------------------------------------------------------



def gen_bw_map(img):

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    kernel = np.ones((2*0+1, 2*0+1), np.uint8)
    obstacle_map = cv2.dilate(255 - binary, kernel)
    return binary, (obstacle_map > 0).astype(np.uint8)



def plan_path(robot, known_map):

    bfs_path_planning(robot, known_map)
    #print(123, robot['planned_path'])
    robot['action'] = 'rotate'
    return robot



def update_robot(robot, known_map, obstacle_map):

    known_map = update_known_map(robot, known_map, obstacle_map)

    if robot['action'] == 'scan':
        done, known_map = initial_scan(robot, 0, known_map, obstacle_map)
        if done:
            robot = plan_path(robot, known_map)
            #print(124, robot['planned_path'])
    
    elif robot['action'] == 'rotate':
        robot = plan_path(robot, known_map)
        execute_move(robot, obstacle_map)

    elif robot['action'] == 'move':
        robot = plan_path(robot, known_map)
        execute_move(robot, obstacle_map)
    
    return robot, known_map



def visualize(robot, known_map, true_map, frame):

    cv2.imwrite(f"frames/map3.png", known_map)

    vis_img = cv2.cvtColor(known_map.copy(), cv2.COLOR_GRAY2BGR)

    if limited_map is not None:

        limited_layer = cv2.cvtColor(limited_map.copy(), cv2.COLOR_GRAY2BGR)
        limited_layer[:] = (100, 100, 100)

        mask = limited_map < 200
        vis_img[mask] = cv2.addWeighted(vis_img[mask], 0, limited_layer[mask], 1, 0)


    obstacle_mask = true_map < 128
    vis_img[obstacle_mask] = (0, 0, 255)

    for i in range(0, len(robot['planned_path'])):

        start = None
        end = None
        if (i == 0):
            start = (robot['pos'][0], robot['pos'][1])
            end = (robot['planned_path'][i][0], robot['planned_path'][i][1])
        else:
            start = (int(robot['planned_path'][i-1][0]), int(robot['planned_path'][i-1][1]))
            end = (int(robot['planned_path'][i][0]), int(robot['planned_path'][i][1]))
        if 0 <= start[0] < MAP_SIZE and 0 <= start[1] < MAP_SIZE and \
            0 <= end[0] < MAP_SIZE and 0 <= end[1] < MAP_SIZE:
            cv2.line(vis_img, start, end, (0, 165, 255), 1)

        cv2.putText(
            vis_img,
            f"{int(end[0]), int(end[1])}",
            end, 
            cv2.FONT_HERSHEY_SIMPLEX,
            0.2,
            (0, 0, 0),
            1,
            cv2.LINE_AA
        )

    for i in range(1, len(robot['path'])):
        start = (int(robot['path'][i-1][0]), int(robot['path'][i-1][1]))
        end = (int(robot['path'][i][0]), int(robot['path'][i][1]))
        cv2.line(vis_img, start, end, (0, 255, 0), 1)

        

    end_x = int(robot['pos'][0] + (ROBOT_RADIUS+3) * math.cos(robot['heading']))
    end_y = int(robot['pos'][1] + (ROBOT_RADIUS+3) * math.sin(robot['heading']))
    cv2.line(vis_img, (int(robot['pos'][0]), int(robot['pos'][1])),(end_x, end_y),(255, 200, 200), 2)
    cv2.circle(vis_img, (int(robot['pos'][0]), int(robot['pos'][1])),  ROBOT_RADIUS, (255, 0, 0), -1)
    
    cv2.circle(vis_img, (int(robot['goal'][0][0]), int(robot['goal'][0][1])), 5, (0, 0, 255), -1)

    cv2.putText(
        vis_img,
        f"Frame: {frame}, action: {robot['action']}",
        (10, 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 0, 0),
        1,
        cv2.LINE_AA 
    )

    return vis_img



def main():

    map_img = np.ones((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8) * 255
    for _ in range(50):
        cx, cy = random.randint(0, 100), random.randint(70, 200)
        cv2.circle(map_img, (cx, cy), random.randint(5, 20), (0, 0, 0), -1)
    for _ in range(50):
        cx, cy = random.randint(200, 250), random.randint(70, 200)
        cv2.circle(map_img, (cx, cy), random.randint(5, 20), (0, 0, 0), -1)
    
    start_pos = (random.randint(15, 20), random.randint(15, 20))
    goal_pos = (random.randint(220, 230), random.randint(220, 230))
    
    cv2.circle(map_img, start_pos, ROBOT_RADIUS+5, (255, 255, 255), -1)
    cv2.circle(map_img, goal_pos, 10, (255, 255, 255), -1)
    
    true_map, obstacle_map = gen_bw_map(map_img)

    

    known_map = np.ones((MAP_SIZE, MAP_SIZE), dtype=np.uint8) * 255
    robot = create_robot(start_pos, goal_pos)

    #print()
    
    os.makedirs("frames", exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_out = cv2.VideoWriter(VIDEO_FILE, fourcc, FRAME_RATE, (MAP_SIZE, MAP_SIZE))
    
    frame_count = 0
    max_frames = 1500
    
    while frame_count < max_frames and (robot['isReturn'] or np.linalg.norm(robot['pos'] - robot['goal'][0]) > ROBOT_RADIUS):
        if (np.linalg.norm(robot['pos'] - robot['goal'][0]) < ROBOT_RADIUS):
            print(1, robot['goal'])
            robot['goal'] = np.delete(robot['goal'], 0, 0)
            robot['isReturn'] = False
            robot['planned_path'] = np.array([robot['goal'][0]])
            print(1, robot['goal'])

        robot, known_map = update_robot(robot, known_map, obstacle_map)
        vis_img = visualize(robot, known_map, true_map, frame_count)
        video_out.write(vis_img)
        frame_count += 1
        print(frame_count)

    for _ in range(30):
        video_out.write(vis_img)

    for i in range(MAP_SIZE):
        for j in range(MAP_SIZE):
            if (known_map[i,j] < 200):
                #print((i, j))
                pass
    
    video_out.release()
    print(f"视频保存为: {VIDEO_FILE}")

if __name__ == "__main__":
    main()