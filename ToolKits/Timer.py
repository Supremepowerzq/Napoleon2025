import time

def precise_delay(target_duration: float) -> None:
    """
    精确延迟指定的持续时间，结合使用 time.sleep 和忙等待来提高延迟精度。

    参数:
    - target_duration (float): 目标延迟时间，单位为秒。
    """
    start_time = time.perf_counter()
    while True:
        current_time = time.perf_counter()
        elapsed = current_time - start_time
        remaining = target_duration - elapsed
        if remaining <= 0:
            break
        elif remaining > 0.01:  # 如果剩余时间大于10毫秒，则sleep大部分时间
            time.sleep(remaining / 2)
        # 在最后10毫秒内，执行忙等待以提高精度

def busy_maintain_target_frequency(target_frequency: float, last_iteration_start: float) -> None:
    """
    维持操作以目标频率执行。如果操作完成得太快，则使用精确延迟直到达到目标周期。

    参数:
    - target_frequency (float): 目标频率，单位为Hz。
    - last_iteration_start (float): 上一次迭代开始的时间点，使用 time.perf_counter() 的返回值。
    """
    elapsed_time = time.perf_counter() - last_iteration_start
    target_period = 1 / target_frequency
    if elapsed_time < target_period:
        precise_delay(target_period - elapsed_time)

def maintain_target_frequency(target_frequency: float, last_iteration_start: float) -> None:
    """
    维持操作以目标频率执行。如果操作完成得太快，则使用 time.sleep 直到达到目标周期。

    参数:
    - target_frequency (float): 目标频率，单位为Hz。
    - last_iteration_start (float): 上一次迭代开始的时间点，使用 time.perf_counter() 的返回值。
    """
    elapsed_time = time.perf_counter() - last_iteration_start
    target_period = 1 / target_frequency
    if elapsed_time < target_period:
        time.sleep(target_period - elapsed_time)
