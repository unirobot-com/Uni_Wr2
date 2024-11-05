# 安装RPi.GPIO库
import RPi.GPIO as GPIO

class HallEncoder:
    def __init__(self, pinA, pinB):
        self.pinA = pinA
        self.pinB = pinB
        self.count = 0

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pinA, GPIO.IN)
        GPIO.setup(self.pinB, GPIO.IN)
        # GPIO.add_event_detect(self.pinA, GPIO.BOTH, callback=self._callback_A)
        # GPIO.add_event_detect(self.pinB, GPIO.BOTH, callback=self._callback_B)
        GPIO.add_event_detect(self.pinA, GPIO.BOTH, callback=self.interrupt_a)
        self.interrupt_a_module = False

    def interrupt_a(self,channel):
        self.interrupt_a_module = False
        self.read_encoder()

    def _callback_A(self, channel):
        if GPIO.input(self.pinA) ==1:
            if GPIO.input(self.pinB) == 1:
                self.count += 1
            else:
                self.count -= 1
        else:
            if GPIO.input(self.pinB) ==1:
                self.count -= 1
            else:
                self.count += 1
        print(self.count)

        # if GPIO.input(self.pinB):
        #     self.count += 1
        # else:
        #     self.count -= 1

    def _callback_B(self, channel):
        print("B:", GPIO.input(self.pinB))

    def read(self):
        return self.count

    def read_encoder(self):
        if GPIO.input(self.pinA) ==1:
            if GPIO.input(self.pinB) == 1:
                self.count += 1
            else:
                self.count -= 1
        else:
            if GPIO.input(self.pinB) ==1:
                self.count -= 1
            else:
                self.count += 1
        print(self.count)

if __name__ == '__main__':
    encoder = HallEncoder(pinA=16, pinB=13)

    while True:
        nothing_to_do = 1
        # encoder.read_encoder()
        # print(encoder.read())