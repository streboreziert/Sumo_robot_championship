from machine import Pin, PWM, ADC, I2C
import neopixel
import time
import vl53l0x
import robot
from button import Button

# NeoPixel LEDs
np = neopixel.NeoPixel(Pin(14), 6)

# Buttons
btn1 = Button(26)
btn2 = Button(27)

# IR object sensors
ir = PWM(Pin(5), freq=38000, duty_u16=0x8000)
senL = Pin(35, Pin.IN, Pin.PULL_UP)
senFL = Pin(34, Pin.IN, Pin.PULL_UP)
senR = Pin(4, Pin.IN, Pin.PULL_UP)
senFR = Pin(23, Pin.IN, Pin.PULL_UP)

# Buzzer
buzzer = PWM(Pin(25), duty_u16=0)

# Floor sensors
flrL = ADC(Pin(36))
flrR = ADC(Pin(39))
flrL.atten(ADC.ATTN_11DB)
flrR.atten(ADC.ATTN_11DB)

# I2C
i2cTof = I2C(0,sda=Pin(32),scl=Pin(33),freq=400000)
i2cImu = I2C(1,sda=Pin(21),scl=Pin(22),freq=400000)

# ToF sensor
#tof = vl53l0x.VL53L0X(i2cTof)
#tof.set_Vcsel_pulse_period(tof.vcsel_period_type[0], 18)
#tof.set_Vcsel_pulse_period(tof.vcsel_period_type[1], 14)
#tof.start()

# IMU + motors
rob = robot.Robot(i2cImu)

run = 0
targetAngle = 0.0

while True:
    #if(btn1.getEvent() == Button.PRESS):
    #    print(tof.read())
    
    #print(rob.imu.readAngle())    
    #time.sleep_ms(50)
    #pass
    event = btn1.getEvent()
    if(event == Button.PRESS):
        run = run ^ 1
        if(run == 1):
            rob.imu.resetAngle()
            
    if(run):
        errAngle = targetAngle - rob.imu.readAngle()
        err = int(errAngle * 500)
        rob.drive(-10000 + err, -10000 - err)
         
        if (flrL.read() > 2000 or flrR.read() > 2000):
            rob.turn(80, 100000)
            rob.imu.resetAngle()
        
        if (senL.read() == 0):
            np[0] = (0,255,0)
    else:
        rob.stop()
         
