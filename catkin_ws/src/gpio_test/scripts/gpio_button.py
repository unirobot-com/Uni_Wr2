from gpiozero import Button
from signal import pause

class GPIOInterruptHandler:
    def __init__(self, pin_list):
        self.buttons = []
        for pin in pin_list:
            # 不再明确设置 pull_up 参数
            btn = Button(pin)
            btn.when_pressed = self.when_pressed
            btn.when_released = self.when_released
            self.buttons.append(btn)

    def when_pressed(self):
        print(f"Detected a signal change to HIGH on one of the GPIO pins!")

    def when_released(self):
        print(f"Detected a signal change to LOW on one of the GPIO pins!")

    def run(self):
        print("Running. Press CTRL+C to exit.")
        pause()

# 使用示例
if __name__ == "__main__":
    # 避免使用具有物理上拉电阻的引脚，或确保你的应用可以在这些引脚默认的上拉状态下工作
    pins = [4, 17]  # 修改为不含物理上拉电阻的GPIO引脚
    gpio_handler = GPIOInterruptHandler(pins)
    gpio_handler.run()