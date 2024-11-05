from gpiozero import Button
from signal import pause

class EncoderHandler:
    def __init__(self, pin_a, pin_b):
        self.pin_a = Button(pin_a)
        self.pin_b = Button(pin_b)
        self.position = 0

        # 监听A相和B相的状态变化
        self.pin_a.when_pressed = self.handle_movement
        self.pin_a.when_released = self.handle_movement
        self.pin_b.when_pressed = self.handle_movement
        self.pin_b.when_released = self.handle_movement

    def handle_movement(self):
        if self.pin_a.is_pressed and self.pin_b.is_pressed:
            # 此时A相和B相状态一致，这种情况可根据具体情况处理
            pass
        elif self.pin_a.is_pressed:
            # A相先于B相变化，且为高电平，做为电机正转的示例
            self.position += 1
        elif self.pin_b.is_pressed:
            # B相先于A相变化，且为高电平，做为电机反转的示例
            self.position -= 1

        print(f"Current position: {self.position}")

    def run(self):
        print("Monitoring encoder movement. Press CTRL+C to exit.")
        pause()

# 使用示例
if __name__ == "__main__":
    encoder_handler = EncoderHandler(4, 17)
    encoder_handler.run()