import asyncio
import websockets
import json
import time
import numpy as np
import cv2
import math
from math import sin, cos, atan2, pi, sqrt
import random
import os
from collections import deque

MAP_TRUE_SIZE = 5.0
MAP_SIZE = 250
SENSOR_RANGE = 50
ROTATION_STEP = math.pi / 18
OFFSET = 50 
MOVE_STEP = 2
PIXEL_SIZE = MAP_TRUE_SIZE/MAP_SIZE
map_data = None
robot = None
known_map = None
limited_map = None
warning_map = None



# Moving -----------------------------------------------------------------



def rotate_step(robot, target_angle):
    #print("angle diff", target_angle, robot['yaw'])

    angle_diff = (target_angle - robot['yaw']) % (2 * math.pi)
    
    if angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    
    if abs(angle_diff) > ROTATION_STEP:
        if (angle_diff > 0):
            robot['key'] = 'right'
        else:
             robot['key'] = 'left'
        return False
        
    else:
        robot['yaw'] = target_angle
        return True

def move_step(robot, target_pos):

    angle = robot['yaw']
    real_vec = np.array([cos(angle), sin(angle)])
    target_vec = target_pos - robot['pos']
    target_dist = np.linalg.norm(target_vec)
    
    if target_dist > MOVE_STEP:

        if (np.dot(real_vec, target_vec) < 0):
            robot['isOut'] = True

        robot['key'] = 'forward'
        return True
    
    else:
        robot['pos'] = [int(target_pos[0]), int(target_pos[1])]
        robot['path'].append(robot['pos'].copy())
        if (len(robot['planned_path']) >= 2):
            robot['planned_path'] = np.delete(robot['planned_path'], 0, 0)
        else:
            robot['planned_path'] = np.delete(robot['planned_path'], 0, 0)
            robot['goal_pos'] = np.delete(robot['goal_pos'], 0, 0)
            robot['action'] = 'stop'
            robot['key'] = 'stop'
            print(f'arrived at point {target_pos}')
        return True
    
def execute_move(robot):
    
    target_pos = robot['planned_path'][0]
    #print(robot['planned_path'])
    target_vec = target_pos - robot['pos']
    target_dir = math.atan2(target_vec[1], target_vec[0])

    robot['path'].append([robot['pos'][0], robot['pos'][1]])
    if len(robot['path']) > 1000:
        robot['path'][0:-1] = robot['path'][1:-2]
        

    if robot['action'] == 'rotate':
        done = rotate_step(robot, target_dir)
        #print(done)
        if done:
            robot['action'] = 'move'
    
    if robot['action'] == 'move':
        done = move_step(robot, target_pos)
        if robot['isReturn']:
            print('robot: ', robot['pos'],', target original point: ', target_pos)
        
        return done
    


# pathes finding -----------------------------------------------------------



def create_limited_area_map(known_map, size):

    known_map = np.uint8(known_map)

    kernel = np.ones((size, size), np.uint8)
    
    inverted_map = 255 - known_map  # 反转，使障碍=255，自由=0
    limited_area_map = cv2.dilate(inverted_map, kernel, iterations=3)
    limited_area_map = 255 - limited_area_map 

    return limited_area_map



def line_crosses_limited_area(p1, p2, limited_map):
    x1, y1 = p1[0], p1[1]
    x2, y2 = p2[0], p2[1]

    # 计算线段方向和步进参数
    dx = x2 - x1
    dy = y2 - y1
    length = max(abs(dx), abs(dy))  # 使用最大分量确保覆盖所有像素
    
    if length == 0:  # 两点重合
        return limited_map[int(y1), int(x1)] < 200
    
    # 标准化步进量
    dx /= length
    dy /= length

    # 遍历线段上的所有点
    x, y = x1, y1
    for _ in range(int(length) + 1):  # +1确保包含终点
        intX = int(round(x))
        intY = int(round(y))
        
        # 检查是否越界（可选，取决于输入保证）
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
                
            # 标记访问
            accumulated_map[i, j] = current_depth
            dist = np.linalg.norm(np.array([i, j])-goal_pos)
            if line_crosses_limited_area(point_before, [i, j], warning_map):
                dist += 1000
            next_points.append([dist, [i, j]])


    next_points.sort()
    #print(point_before, len(next_points))
    
    # 递归搜索
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

    global warning_map


    if not line_crosses_limited_area(robot['planned_path'][0], robot['pos'], limited_map):
        return

    cv2.imwrite(f"frames/map5.png", warning_map)

    global accumulated_map
    accumulated_map = np.zeros_like(known_map, dtype=np.uint8)
    accumulated_map += 255

    global times
    times = 0

    start_pos = robot['pos']
    goal_pos = robot['goal_pos'][0]
    heading = robot['yaw']
    
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
        #cv2.imwrite(f"frames/map4.png", np.clip(accumulated_map*30, 0, 255)) 
        
        if (found_points == None):
            found_points = []
        
        if isFound:
            robot['planned_path'] = np.array([ray_points[i][1]] + found_points + [goal_pos])
            #print(122, ray_points[i][1], robot['planned_path'])
            #print(robot['pos'], robot['planned_path'])
            return
        
    #cv2.imwrite(f"frames/map4.png", np.clip(accumulated_map*30, 0, 255))
    
    if not (robot['histroy_pos'][0] == robot['pos'][0] and robot['histroy_pos'][1] == robot['pos'][1]):
        robot['histroy_pos'] = robot['pos'].copy()
        bfs_path_planning(robot, known_map)
    else:
        #print(robot['planned_path'], robot['path'])
        print('return')
        if len(robot['path']) > 0 and not robot['isReturn']:
            robot['goal_pos'] = np.array([robot['path'][0], robot['goal_pos'][0]])
            print('goal: ', robot['goal_pos'])
            robot['isReturn'] = True


# Painting Obstacles --------------------------------------------------



def update_known_map(robot, known_map):

    dist = robot['dist']*50
    #print(dist)

    if (dist > SENSOR_RANGE):
        return known_map
    
    x = int(robot['pos'][0] + dist * math.cos(robot['yaw']))
    y = int(robot['pos'][1] + dist * math.sin(robot['yaw']))

    if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
        known_map[y, x] = 0
    
    for dx in range(-robot['radius']-1, robot['radius']+2):
        for dy in range(-robot['radius']-1, robot['radius']+2):
            x, y = int(robot['pos'][0]) + dx, int(robot['pos'][1]) + dy
            if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
                dist_to_robot = math.sqrt(dx**2 + dy**2)
                if dist_to_robot <= robot['radius'] + 0.5:
                    known_map[y, x] = 255

    global limited_map

    limited_map = create_limited_area_map(known_map, 4)

    global warning_map

    warning_map = create_limited_area_map(known_map, 10)
    
    return known_map


def plan_path(robot, known_map):

    bfs_path_planning(robot, known_map)
    #print(123, robot['planned_path'])
    robot['action'] = 'rotate'
    return robot


def update_robot(robot, known_map):

    known_map = update_known_map(robot, known_map)

    #print(len(robot['goal_pos']) > 0)

    if (len(robot['goal_pos']) > 0):

        if robot['action'] == 'stop':
            print('start rebuilding path')
            robot['planned_path'] = robot['goal_pos'].copy()
            robot['action'] = 'rotate'
            pass

        elif robot['action'] == 'rotate':
            robot = plan_path(robot, known_map)
            execute_move(robot)

        elif robot['action'] == 'move':
            robot = plan_path(robot, known_map)
            execute_move(robot)
    
    return robot, known_map


def visualize(robot, known_map):

    cv2.imwrite(f"frames/map3.png", known_map)

    vis_img = cv2.cvtColor(known_map.copy(), cv2.COLOR_GRAY2BGR)

    if limited_map is not None:
        
        # 创建灰色叠加层
        limited_layer = cv2.cvtColor(limited_map.copy(), cv2.COLOR_GRAY2BGR)
        limited_layer[:] = (100, 100, 100)  # 设置为灰色

        
        
        # 只叠加limited_map中非零的部分
        mask = limited_map < 200

        if len(limited_layer[mask]) > 0:
            vis_img[mask] = cv2.addWeighted(vis_img[mask], 0, limited_layer[mask], 1, 0)


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
            vis_img,  # 目标图像
            f"{int(end[0]), int(end[1])}",  # 显示的文本
            end, 
            cv2.FONT_HERSHEY_SIMPLEX,  # 字体
            0.3,  # 字体大小
            (0, 0, 0),  # 白色文本
            1,  # 线宽
            cv2.LINE_AA  # 抗锯齿
        )

    for i in range(1, len(robot['path'])):
        start = (int(robot['path'][i-1][0]), int(robot['path'][i-1][1]))
        end = (int(robot['path'][i][0]), int(robot['path'][i][1]))
        cv2.line(vis_img, start, end, (0, 255, 0), 1)

    end_x = int(robot['pos'][0] + (robot['radius']+3) * math.cos(robot['yaw']))
    end_y = int(robot['pos'][1] + (robot['radius']+3) * math.sin(robot['yaw']))
    cv2.line(vis_img, (int(robot['pos'][0]), int(robot['pos'][1])),(end_x, end_y),(255, 200, 200), 2)
    cv2.circle(vis_img, (int(robot['pos'][0]), int(robot['pos'][1])),  robot['radius'], (255, 0, 0), -1)
    
    for i in range(len(robot['goal_pos'])):
        cv2.circle(vis_img, (int(robot['goal_pos'][0][0]), int(robot['goal_pos'][0][1])), 5, (0, 0, 255), -1)

    return vis_img

def img_to_map(vis_img):

    MAP_SIZE = vis_img.shape[0]
    rgba_img = cv2.cvtColor(vis_img, cv2.COLOR_BGR2RGBA)
  
    rgba_uint32 = rgba_img.view(np.uint32).reshape((MAP_SIZE, MAP_SIZE))

    return rgba_uint32


async def handle_data(websocket):

    global robot

    print("控制客户端已连接")
    try:
        async for message in websocket:
            data = json.loads(message)

            if data.get("type") == "init":

                robot["true_pos"][0] = float(data.get("x", 0))
                robot["true_pos"][1] = float(data.get("y", 0))
                robot["yaw"] = float(data.get("yaw", 0))*np.pi/180

                robot["pos"][0] = int(robot["true_pos"][0]/PIXEL_SIZE)
                robot["pos"][1] = int(robot["true_pos"][1]/PIXEL_SIZE)

                x = robot["pos"][0]
                y = robot["pos"][1]
                    
                await websocket.send(json.dumps({
                    "type": "log",
                    "message": f"Robot initialized at ({x}, {y})"
                }))
                    

            if data.get("type") == "robot":

                robot["true_pos"][0] = float(data.get("x", robot["true_pos"][0]))
                robot["true_pos"][1] = float(data.get("y", robot["true_pos"][1]))

                robot["pos"][0] = int(robot["true_pos"][0]/PIXEL_SIZE)
                robot["pos"][1] = int(robot["true_pos"][1]/PIXEL_SIZE)

                robot["yaw"] = float(data.get("yaw", robot["yaw"]))*np.pi/180
                
                robot["dist"] = float(data.get("depth", robot["dist"]))

                    
            if data.get("type") == "target":

                a = [data.get("x", 0), data.get("y", 0)]

                if (robot["goal_pos"] is not None):
                    if (robot["goal_pos"].size < 1):
                        robot["goal_pos"] = np.array([np.array(a, dtype=int)])
                        robot['action'] = 'stop'
                    else:
                        if not (a[0] == robot["goal_pos"][-1][0] and a[1] == robot["goal_pos"][-1][1]):
                            robot["goal_pos"] = np.array([np.array(a, dtype=int)])
                            robot['action'] = 'stop'
                            
                            
                            
                
    except websockets.exceptions.ConnectionClosed:
        print("控制客户端断开连接")


async def continuous_action(websocket):

    global robot
    global known_map

    while True:
        #try:
        start_time = time.time()

        #if robot['key'] == robot['key_before']:  
                
        robot, known_map = update_robot(robot, known_map)
        vis_img = visualize(robot, known_map)
        map_data = img_to_map(vis_img)
                    
        await websocket.send(json.dumps({
            "type": "map_update",
            "map": map_data.tolist(),
            "action": robot["key"],
            "goal": len(robot['goal_pos']) < 1
        }))


        elapsed = time.time() - start_time
        await asyncio.sleep(max(0, 0.1 - elapsed))
        '''
        except (websockets.exceptions.ConnectionClosed, websockets.exceptions.WebSocketException) as e:
            print(f"连接已关闭或发生错误: {e}")
            break
        except Exception as e:
            print(f"发生意外错误: {e}")
            break
        '''
        await asyncio.sleep(max(0, 0.1 - elapsed))

        



async def handler(websocket):

    control_task = asyncio.create_task(handle_data(websocket))
    movement_task = asyncio.create_task(continuous_action(websocket))
    
    # 等待任一任务完成
    done, pending = await asyncio.wait(
        [control_task, movement_task],
        return_when=asyncio.FIRST_COMPLETED
    )
    
    # 取消未完成的任务
    for task in pending:
        task.cancel()



async def main():

    global robot
    global known_map

    robot = {
        'pos': [OFFSET, OFFSET],
        'dist': 0.0,
        'yaw': 0.0,
        'true_pos': np.array([OFFSET*PIXEL_SIZE, OFFSET*PIXEL_SIZE], dtype=float),
        'goal_pos': np.array([]),
        'path': [np.array([OFFSET, OFFSET], dtype=int)],
        'planned_path': np.array([]),

        'action': 'stop',
        'key': 'stop',
        'key_before': 'stop',
        'isReturn': False,
        'histroy_pos': [0, 0],
        'isOut' : False,
        'radius': 4
    }

    #print(robot['planned_path'])

    known_map = np.ones((MAP_SIZE, MAP_SIZE), dtype=np.uint8) * 255

    #print(known_map)
    
    # 启动WebSocket服务器
    async with websockets.serve(handler, "localhost", 8765):
        print("控制服务器已启动: ws://localhost:8765")
        await asyncio.Future()  # 永久运行

if __name__ == "__main__":
    asyncio.run(main())



