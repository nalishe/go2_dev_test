'''
    # 鍵盤控制說明：
    w:向前移動
    e:向後並向左轉移動 (普通模式下)
    a:向左轉
    d:向右轉
    q:向前並向左轉移動 (普通模式下)
    s:向後移動
    c:向後並向左轉移動 (全向模式下e相對)
    z:向前並向右轉移動 (全向模式下q相對)

    hold shift進入全向模式 可以側向移動

    W:向前移動
    E:向左移動
    A:向後並向左移動
    D:向後並向右移動
    Q:向右移動
    S:停止
    C:向前並向左移動
    Z:向前並向右移動

    速度調整
    r:增加最大速度和轉向速度10%
    t:減少最大速度和轉向速度10%
    f:增加線性速度10%
    g:減少線性速度10%
    v:增加轉向速度10%
    b:減少轉向速度10%

    h:打招呼
    j:前跳
    k:伸懶腰
    n:坐下
    m:從坐下恢復
    y:跳舞1
    u:跳舞2
'''

# 機器人運動API對應ID表
ROBOT_SPORT_API_IDS = {
    'DAMP': 1001,              # 阻尼模式
    'BALANCESTAND': 1002,     # 平衡站立
    'STOPMOVE': 1003,         # 停止移動
    'STANDUP': 1004,          # 站起來
    'STANDDOWN': 1005,        # 蹲下
    'RECOVERSTAND': 1006,     # 站立復原
    'EULER': 1007,            # 歐拉角控制
    'MOVE': 1008,             # 移動
    'SIT': 1009,              # 坐下
    'RISESIT': 1010,          # 起身坐下
    'SWITCHGAIT': 1011,       # 切換步態
    'TRIGGER': 1012,          # 觸發動作
    'BODYHEIGHT': 1013,       # 設定身體高度
    'FOOTRAISEHIGH': 1014,    # 抬腳高度
    'SPEEDLEVEL': 1015,       # 速度等級
    'HELLO': 1016,            # 打招呼
    'STRETCH': 1017,          # 伸展
    'TRAJECTORYFOLLOW': 1018, # 路徑跟隨
    'CONTINUOUSGAIT': 1019,   # 連續步態
    'CONTENT': 1020,          # 內容模式
    'WALLOW': 1021,           # 打滾
    'DANCE1': 1022,           # 跳舞1
    'DANCE2': 1023,           # 跳舞2
    'GETBODYHEIGHT': 1024,    # 取得身體高度
    'GETFOOTRAISEHIGH': 1025, # 取得抬腳高度
    'GETSPEEDLEVEL': 1026,    # 取得速度等級
    'SWITCHJOYSTICK': 1027,   # 切換搖桿
    'POSE': 1028,             # 姿態控制
    'SCRAPE': 1029,           # 刮地
    'FRONTFLIP': 1030,        # 前空翻
    'FRONTJUMP': 1031,        # 前跳
    'FRONTPOUNCE': 1032       # 前撲
}

# 特殊動作對應鍵盤按鍵
sportModel = {
    'h': ROBOT_SPORT_API_IDS['HELLO'],         # 打招呼
    'j': ROBOT_SPORT_API_IDS['FRONTJUMP'],     # 前跳
    'k': ROBOT_SPORT_API_IDS['STRETCH'],       # 伸懶腰
    'n': ROBOT_SPORT_API_IDS['SIT'],           # 坐下
    'm': ROBOT_SPORT_API_IDS['RISESIT'],       # 從坐下恢復
    'y': ROBOT_SPORT_API_IDS['DANCE1'],        # 跳舞1
    'u': ROBOT_SPORT_API_IDS['DANCE2'],        # 跳舞2
    'o': ROBOT_SPORT_API_IDS['STANDDOWN'],     # 蹲下
    'p': ROBOT_SPORT_API_IDS['STANDUP']        # 站起來
}

# 移動指令對應鍵盤按鍵
moveBindings = {
    'w':(1, 0, 0, 0), # x*1, y*0, z*0, th*0 (前進)
    'e':(1, 0, 0, -1),
    'a':(0, 0, 0, 1),
    'd':(0, 0, 0, -1),
    'q':(1, 0, 0, 1),
    's':(-1, 0, 0, 0),
    'c':(-1, 0, 0, 1),
    'z':(-1, 0, 0, -1),
    'E':(1, -1, 0, 0),
    'A':(0, 1, 0, 0),
    'D':(0, -1, 0, 0),
    'Q':(1, 1, 0, 0),
    'S':(-1, 0, 0, 0),
    'C':(-1, -1, 0, 0),
    'Z':(-1, 1, 0, 0),
}

# 速度調整對應鍵盤按鍵
speedBindings = {
    'r':(1.1, 1.1), # 增加最大速度和轉向速度10%
    't':(.9, .9),   # 減少最大速度和轉向速度10%
    'f':(1.1, 1),   # 增加線性速度10%
    'g':(.9, 1),    # 減少線性速度10%
    'v':(1, 1.1),   # 增加轉向速度10%
    'b':(2, .9)     # 減少轉向速度10%
}

# 操作說明訊息
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   Q    W    E
   A    S    D
   Z    X    C

anything else : stop

r/t : increase/decrease max speeds by 10%
f/g : increase/decrease only linear speed by 10%
v/b : increase/decrease only angular speed by 10%

h:Great
j:FrontJump
k:Stretch
n:Sit down
m:stand up from sit
y:Dance1
u:Dance2

CTRL-C to quit
"""

# 匯入 ROS2 Python 函式庫
import rclpy
from rclpy.node import Node

import termios
import sys
import tty
import threading
import json

from unitree_api.msg import Request

# 鍵盤遙控節點
class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_ctrl_keyboard')
        # 建立 Publisher，發佈 Request 到 Unitree API
        self.pub = self.create_publisher(Request, '/api/sport/request', 10)

        # 宣告速度參數
        self.declare_parameter('speed', 0.2)
        self.declare_parameter('angular', 0.5)
        self.speed = self.get_parameter('speed').value
        self.angular = self.get_parameter('angular').value

    # 發佈請求訊息
    def publish(self, api_id, x = 0.0, y = 0.0, z = 0.0):
        req = Request()
        req.header.identity.api_id = api_id
        js = {'x':x, 'y':y, 'z':z}
        req.parameter = json.dumps(js)
        self.pub.publish(req)
        

# 取得鍵盤輸入

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# 主程式進入點
def main():
    print(msg)
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    teleopNode = TeleopNode()
    spinner = threading.Thread(target=rclpy.spin, args=(teleopNode, ))
    spinner.start()
    

    try:
        while True:
            key = getKey(settings)

            if key == '\x03':
                # teleopNode.publish(ROBOT_SPORT_API_IDS['BALANCESTAND'])
                break
            elif key in sportModel.keys():
                teleopNode.publish(sportModel[key])
            
            elif key in moveBindings.keys():
                x_bind = moveBindings[key][0]
                y_bind = moveBindings[key][1]
                z_bind = moveBindings[key][2]
                th_bind = moveBindings[key][3]

                teleopNode.publish(ROBOT_SPORT_API_IDS['MOVE'],
                                   x=x_bind * teleopNode.speed,
                                   y=y_bind * teleopNode.speed,
                                   z=th_bind * teleopNode.angular)
            
            elif key in speedBindings.keys():
                s_bind = speedBindings[key][0]
                a_bind = speedBindings[key][1]

                teleopNode.speed = s_bind * teleopNode.speed
                teleopNode.angular = a_bind * teleopNode.angular

                # print("current speed = %.2f, angular = %.5f" %(teleopNode.speed, teleopNode.angular ))
                
            else:
                teleopNode.publish(ROBOT_SPORT_API_IDS['BALANCESTAND'])
                
    finally:
        teleopNode.publish(ROBOT_SPORT_API_IDS['BALANCESTAND'])
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()