import asyncio
import websockets
import json
import random
import time
import math
import numpy as np
import cv2

# 地图参数
MAP_SIZE = 250  # 地图尺寸
OFFSET = 50     # 初始偏移量
map_img = None  # 地图图像
SENSOR_RANGE = 50  # 传感器范围
ROBOT_RADIUS = 4   # 机器人半径
SENSOR_ERROR = 1   # 传感器误差

# 机器人状态
robot = {
    'pos': [OFFSET, OFFSET],  # 像素坐标位置
    'yaw': 0.0,               # 朝向角度
    'actual_pos': np.array([OFFSET*0.02, OFFSET*0.02], dtype=float),  # 实际位置(米)
    'dist': 0,                # 传感器距离
    'move_state': 'stop',     # 移动状态
    'last_update': time.time()  # 最后更新时间
}

async def handle_control(websocket):

    print("控制客户端已连接")
    try:
        async for message in websocket:
            data = json.loads(message)
            command = data.get("command")
            
            if command in ["forward", "backward", "left", "right", "stop"]:
                robot['move_state'] = command
            else:
                print(f"收到无效命令: {command}")
                
    except websockets.exceptions.ConnectionClosed:
        print("控制客户端断开连接")

async def continuous_movement(websocket):

    while True:
        try:
            start_time = time.time()

            if robot['move_state'] != "stop":  
                rand1 = float(random.randint(-2, 9))/100
                if robot['move_state'] == "forward":
                    robot['actual_pos'][0] += rand1 * math.cos(math.radians(robot['yaw']))
                    robot['actual_pos'][1] += rand1 * math.sin(math.radians(robot['yaw']))
                    robot['pos'][0] = int(robot['actual_pos'][0]*50)
                    robot['pos'][1] = int(robot['actual_pos'][1]*50)
                elif robot['move_state'] == "backward":
                    robot['actual_pos'][0] -= rand1 * math.cos(math.radians(robot['yaw']))
                    robot['actual_pos'][1] -= rand1 * math.sin(math.radians(robot['yaw']))
                    robot['pos'][0] = int(robot['actual_pos'][0]*50)
                    robot['pos'][1] = int(robot['actual_pos'][1]*50)
                elif robot['move_state'] == "left":
                    robot['yaw'] = (robot['yaw'] - float(random.randint(5, 30))) % 360
                elif robot['move_state'] == "right":
                    robot['yaw'] = (robot['yaw'] + float(random.randint(5, 30))) % 360


            distance_sensor(robot, map_img)

            act_img = visualize(robot, map_img)

            map_data = img_to_map(act_img)

            telemetry = {
                "map": map_data.tolist(),
                "depth": round(robot['dist'] * 1000),
                "pos_x": round(robot['actual_pos'][0], 3),
                "pos_y": round(robot['actual_pos'][1], 3),
                "yaw": round(robot['yaw'], 2)
            }

            #print(map_data.shape)
            
            # 尝试发送数据，如果失败则退出循环
            await websocket.send(json.dumps(telemetry))
            #print(f"状态: {robot['move_state']}")
            
            # 控制更新频率
            elapsed = time.time() - start_time
            await asyncio.sleep(max(0, 0.1 - elapsed))
            
        except (websockets.exceptions.ConnectionClosed, websockets.exceptions.WebSocketException) as e:
            print(f"连接已关闭或发生错误: {e}")
            break
        except Exception as e:
            print(f"发生意外错误: {e}")
            break
        await asyncio.sleep(max(0, 0.1 - elapsed))

async def handler(websocket):

    control_task = asyncio.create_task(handle_control(websocket))
    movement_task = asyncio.create_task(continuous_movement(websocket))
    
    # 等待任一任务完成
    done, pending = await asyncio.wait(
        [control_task, movement_task],
        return_when=asyncio.FIRST_COMPLETED
    )
    
    # 取消未完成的任务
    for task in pending:
        task.cancel()

def visualize(robot, known_map):

    vis_img = cv2.cvtColor(known_map.copy(), cv2.COLOR_GRAY2BGR)

    if known_map is not None:
        limited_layer = cv2.cvtColor(known_map.copy(), cv2.COLOR_GRAY2BGR)
        limited_layer[:] = (0, 0, 0)
        
        mask = known_map < 200
        vis_img[mask] = cv2.addWeighted(vis_img[mask], 0, limited_layer[mask], 1, 0)

        # 绘制机器人朝向指示线
        end_x = int(robot['pos'][0] + (ROBOT_RADIUS+3) * math.cos(math.radians(robot['yaw'])))
        end_y = int(robot['pos'][1] + (ROBOT_RADIUS+3) * math.sin(math.radians(robot['yaw'])))

        cv2.line(vis_img, (int(robot['pos'][0]), int(robot['pos'][1])),(end_x, end_y),(255, 200, 200), 2)
        cv2.circle(vis_img, (int(robot['pos'][0]), int(robot['pos'][1])), ROBOT_RADIUS, (255, 0, 0), -1)

    return vis_img

def img_to_map(vis_img):

    MAP_SIZE = vis_img.shape[0]
    rgba_img = cv2.cvtColor(vis_img, cv2.COLOR_BGR2RGBA)
  
    rgba_uint32 = rgba_img.view(np.uint32).reshape((MAP_SIZE, MAP_SIZE))

    #print(hex(rgba_uint32[0, 0]), rgba_img[0, 0])

    return rgba_uint32


def init():

    global map_img
    map_img = cv2.imread('BW_Map1.png', cv2.IMREAD_GRAYSCALE)

    if map_img is None:
        # 如果没有地图文件，创建空白地图
        map_img = np.ones((MAP_SIZE, MAP_SIZE), dtype=np.uint8) * 255
    else:
        # 调整地图大小并二值化
        if map_img.shape[0] != MAP_SIZE or map_img.shape[1] != MAP_SIZE:
            map_img = cv2.resize(map_img, (MAP_SIZE, MAP_SIZE))
        
        _, map_img = cv2.threshold(map_img, 1, 255, cv2.THRESH_BINARY)

    cv2.imwrite('BW_Map11.png', map_img)
    return map_img

def distance_sensor(robot, map_img):

    distance = SENSOR_RANGE
    for r in range(1, SENSOR_RANGE + 1):
        x = int(robot['pos'][0] + (r + ROBOT_RADIUS) * math.cos(math.radians(robot['yaw'])))
        y = int(robot['pos'][1] + (r + ROBOT_RADIUS) * math.sin(math.radians(robot['yaw'])))
        
        # 检查是否超出地图边界
        if not (0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE):
            distance = r - 1
            break
            
        # 检查是否检测到障碍物
        if map_img[y, x] < 200:
            distance = r
            break
    
    # 处理传感器误差
    if (SENSOR_RANGE - SENSOR_ERROR <= distance <= SENSOR_RANGE + SENSOR_ERROR):
        distance = SENSOR_RANGE*5
    
    robot['dist'] = distance/50

async def main():
    """主函数"""
    global map_img
    map_img = init()
    
    # 启动WebSocket服务器
    async with websockets.serve(handler, "localhost", 8754):
        print("控制服务器已启动: ws://localhost:8754")
        await asyncio.Future()  # 永久运行

if __name__ == "__main__":
    asyncio.run(main())