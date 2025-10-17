# from ToolKits.Timer import busy_maintain_target_frequency
# import time
# import tqdm



# if __name__ == "__main__":
#     frequency = 400
#     target_seconds = 5
#     iterations = frequency * target_seconds
#     time_start = time.perf_counter()
#     for _ in tqdm.tqdm(range(iterations)):
#         _time_start = time.perf_counter()
#         busy_maintain_target_frequency(frequency, _time_start)
#     time_cose_1 = time.perf_counter() - time_start

#     time_start = time.perf_counter()
#     for _ in tqdm.tqdm(range(iterations)):
#         time.sleep(1/frequency)
#     time_cose_2 = time.perf_counter() - time_start

#     print(f"目标频率: {frequency}Hz, 目标时间: {target_seconds}s, 总迭代次数: {iterations}")

#     print(f"忙等待迭代时间: {time_cose_1}, 误差率: {round(100*abs(time_cose_1 - target_seconds)/(target_seconds), 3)}%")
#     print(f"原始等待迭代时间: {time_cose_2}, 误差率: {round(100*abs(time_cose_2 - target_seconds)/(target_seconds), 3)}%")

# import pygame
# import sys

# # 初始化Pygame
# pygame.init()

# # 设置窗口大小
# screen = pygame.display.set_mode((800, 600))

# # 设置窗口标题
# pygame.display.set_caption("电机状态监控")

# # 定义颜色
# BLACK = (0, 0, 0)
# WHITE = (255, 255, 255)
# FONT_COLOR = (0, 255, 0)

# # 设置字体
# font = pygame.font.SysFont(None, 24)

# # 模拟的电机状态数据
# motor_states = [
#     {"speed": 0, "position": 0},
#     {"speed": 0, "position": 0},
#     {"speed": 0, "position": 0},
#     {"speed": 0, "position": 0},
#     {"speed": 0, "position": 0},
#     {"speed": 0, "position": 0},
#     {"speed": 0, "position": 0},
# ]

# def update_motor_states():
#     # 这里应该是获取实际电机状态的逻辑
#     # 例如，从硬件接口或网络接口读取最新的状态值
#     # 模拟数据更新
#     for state in motor_states:
#         state["speed"] += 1
#         state["position"] += 1

# def draw_motor_states(screen, motor_states):
#     screen.fill(BLACK)
#     y_offset = 0
#     for i, state in enumerate(motor_states):
#         text = f"电机{i+1} - 速度: {state['speed']}, 位置: {state['position']}"
#         img = font.render(text, True, FONT_COLOR)
#         screen.blit(img, (20, 20 + y_offset))
#         y_offset += 30

# # 游戏主循环
# running = True
# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False

#     update_motor_states()
#     draw_motor_states(screen, motor_states)

#     pygame.display.flip()
#     pygame.time.delay(100)

# pygame.quit()
# sys.exit()
from scipy.spatial.transform import Rotation as R

v = [0, 1, 0] # x=0, y=1, z=0

q = [0.0, 0, 0.3, 0.954] # w=0.954, x=0, y=0, z=0.3

r = R.from_quat(q)
print(r)

print(r.apply(v))
