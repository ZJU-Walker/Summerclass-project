# LCD IIC Example
from machine import I2C,Pin #从 machine 模块导入 I2C、Pin子模块
from vl53l1x import VL53L1X #从 vl53l1x 模块导入 VL53L1X子模块
import time,sensor, image, lcd
lcd.init() # Initialize the lcd screen.
i2c = I2C(sda=Pin("P1"),scl=Pin("P9"),freq=400000)#I2C 初始化：sda--> P0, scl --> P2,频率 40MHz
distance = VL53L1X(i2c) #测距传感器初始化
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # or sensor.GRAYSCALE
sensor.set_framesize(sensor.QQVGA2) # Special 128x160 framesize for LCD Shield.
x = sensor.width() // 2
y = sensor.height() // 2

while True:
    img = sensor.snapshot()
    #img = sensor.snapshot().rotation_corr(180)
    img.draw_string(x, y, str(distance.read()),color = (255, 0, 0),scale=2)
    lcd.display(img) # Take a picture and display the image.
    print("range: mm ", distance.read()) #串口打印距离值
    time.sleep_ms(50)
