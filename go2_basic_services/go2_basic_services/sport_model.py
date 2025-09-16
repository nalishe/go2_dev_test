# 機器人運動API對應ID表
# 用於指定不同運動模式或動作的API識別碼
ROBOT_SPORT_API_IDS = {
    "DAMP": 1001,              # 阻尼模式
    "BALANCESTAND": 1002,     # 平衡站立
    "STOPMOVE": 1003,         # 停止移動
    "STANDUP": 1004,          # 站起來
    "STANDDOWN": 1005,        # 蹲下
    "RECOVERSTAND": 1006,     # 站立復原
    "EULER": 1007,            # 歐拉角控制
    "MOVE": 1008,             # 移動
    "SIT": 1009,              # 坐下
    "RISESIT": 1010,          # 起身坐下
    "SWITCHGAIT": 1011,       # 切換步態
    "TRIGGER": 1012,          # 觸發動作
    "BODYHEIGHT": 1013,       # 設定身體高度
    "FOOTRAISEHIGH": 1014,    # 抬腳高度
    "SPEEDLEVEL": 1015,       # 速度等級
    "HELLO": 1016,            # 打招呼
    "STRETCH": 1017,          # 伸展
    "TRAJECTORYFOLLOW": 1018, # 路徑跟隨
    "CONTINUOUSGAIT": 1019,   # 連續步態
    "CONTENT": 1020,          # 內容模式
    "WALLOW": 1021,           # 打滾
    "DANCE1": 1022,           # 跳舞1
    "DANCE2": 1023,           # 跳舞2
    "GETBODYHEIGHT": 1024,    # 取得身體高度
    "GETFOOTRAISEHIGH": 1025, # 取得抬腳高度
    "GETSPEEDLEVEL": 1026,    # 取得速度等級
    "SWITCHJOYSTICK": 1027,   # 切換搖桿
    "POSE": 1028,             # 姿態控制
    "SCRAPE": 1029,           # 刮地
    "FRONTFLIP": 1030,        # 前空翻
    "FRONTJUMP": 1031,        # 前跳
    "FRONTPOUNCE": 1032       # 前撲
}