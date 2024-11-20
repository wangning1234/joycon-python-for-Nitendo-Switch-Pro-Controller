from pyjoycon.procon import ProController
from pyjoycon import get_PRO_id
import time

# 连接Pro Controller
joycon_id = get_PRO_id()
procon = ProController(*joycon_id)

# 获取状态


while True:
    if time.time() % 1 < 0.2:
        status = procon.get_status()# 每秒的前0.2秒内输出
        print(status)
        status = None
