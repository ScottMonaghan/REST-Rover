import os
import time
import ipaddress
import wifi
import socketpool
import busio
import board
import microcontroller
import terminalio
from digitalio import DigitalInOut, Direction
from adafruit_httpserver.server import HTTPServer
from adafruit_httpserver.request import HTTPRequest
from adafruit_httpserver.response import HTTPResponse
from adafruit_httpserver.methods import HTTPMethod
from adafruit_httpserver.mime_type import MIMEType
import pwmio
from adafruit_motor.motor import DCMotor
from time import monotonic, sleep

MOTOR_DEFAULT_THROTTLE = 0.8
MOTOR_TURNING_THROTTLE = 0.5
MOTOR_MIN_THROTTLE = 0.3
MOTOR_LEFT_FRONT = 0
MOTOR_RIGHT_FRONT = 1
MOTOR_RIGHT_REAR = 2
MOTOR_LEFT_REAR = 3

THROTTLE_SMOOTHING = 0.1

class RobotMotor(object):
    def __init__ (self, dcMotor):
        self.DCMotor = dcMotor
        self.target_throttle = 0.0
    
    def update_throttle(self):
         current_throttle = self.DCMotor.throttle
         if current_throttle is None: #adafruit DCMotor.throttle returns none when set to 0
             current_throttle = 0.0
         throttle_change = abs(THROTTLE_SMOOTHING * (current_throttle - self.target_throttle))
         if self.target_throttle < current_throttle:
             throttle_change *= -1
         smoothed_throttle = current_throttle + throttle_change
         if abs(smoothed_throttle) < MOTOR_MIN_THROTTLE:
            if self.target_throttle == 0:
                 smoothed_throttle = 0
            elif self.target_throttle > 0:
                smoothed_throttle = MOTOR_MIN_THROTTLE
            elif self.target_throttle > 0:
                smoothed_throttle = -1 * MOTOR_MIN_THROTTLE
         self.DCMotor.throttle = smoothed_throttle        
class Robot(object):
    def __init__ (self, motors):
        self.motors = motors
        
    def update_throttles(self):
        for motor in self.motors:
            motor.update_throttle()

        


#  connect to network
print()
print("Connecting to WiFi")

#  onboard LED setup
led = DigitalInOut(board.LED)
led.direction = Direction.OUTPUT
led_api_value = False
led.value = led_api_value

#motors setup
# Songhe BTS7960 43A High Power Motor Driver Module https://www.amazon.com/gp/product/B07TFB22H5
f_l_positive_pwm = pwmio.PWMOut(board.GP0, frequency=25000)
f_l_negative_pwm = pwmio.PWMOut(board.GP1, frequency=25000)
f_r_positive_pwm = pwmio.PWMOut(board.GP2, frequency=25000)
f_r_negative_pwm = pwmio.PWMOut(board.GP3, frequency=25000)
b_r_positive_pwm = pwmio.PWMOut(board.GP4, frequency=25000)
b_r_negative_pwm = pwmio.PWMOut(board.GP5, frequency=25000)
b_l_positive_pwm = pwmio.PWMOut(board.GP6, frequency=25000)
b_l_negative_pwm = pwmio.PWMOut(board.GP7, frequency=25000)


dcMotor_f_l = DCMotor(positive_pwm = f_l_positive_pwm, negative_pwm = f_l_negative_pwm)
dcMotor_f_r = DCMotor(positive_pwm = f_r_positive_pwm, negative_pwm = f_r_negative_pwm)
dcMotor_b_r = DCMotor(positive_pwm = b_r_positive_pwm, negative_pwm = b_r_negative_pwm)
dcMotor_b_l = DCMotor(positive_pwm = b_l_positive_pwm, negative_pwm = b_l_negative_pwm)

robot = Robot([RobotMotor(dcMotor_f_l),RobotMotor(dcMotor_f_r),RobotMotor(dcMotor_b_r),RobotMotor(dcMotor_b_l)])

#  set static IP address
ipv4 =  ipaddress.IPv4Address("192.168.8.42")
netmask =  ipaddress.IPv4Address("255.255.255.0")
gateway =  ipaddress.IPv4Address("192.168.8.1")
wifi.radio.set_ipv4_address(ipv4=ipv4,netmask=netmask,gateway=gateway)
#  connect to your SSID
wifi.radio.connect(os.getenv('CIRCUITPY_WIFI_SSID'), os.getenv('CIRCUITPY_WIFI_PASSWORD'))

print("Connected to WiFi")
pool = socketpool.SocketPool(wifi.radio)
server = HTTPServer(pool)

#  route default static IP
@server.route("/")
def base(request: HTTPRequest):  # pylint: disable=unused-argument
    #  with content type text/html
    with HTTPResponse(request, content_type=MIMEType.TYPE_TXT) as response:
     response.send("hello world")

@server.route("/pb/v1/forward")
def drive_forward(request: HTTPRequest):  # pylint: disable=unused-argument
    #  with content type text/html
    with HTTPResponse(request, content_type=MIMEType.TYPE_TXT) as response:
        response.send("forward")
        robot.motors[MOTOR_LEFT_FRONT].target_throttle = MOTOR_DEFAULT_THROTTLE
        robot.motors[MOTOR_RIGHT_FRONT].target_throttle = MOTOR_DEFAULT_THROTTLE
        robot.motors[MOTOR_RIGHT_REAR].target_throttle = MOTOR_DEFAULT_THROTTLE
        robot.motors[MOTOR_LEFT_REAR].target_throttle = MOTOR_DEFAULT_THROTTLE
        

@server.route("/pb/v1/back")
def drive_back(request: HTTPRequest):  # pylint: disable=unused-argument
    #  with content type text/html
    with HTTPResponse(request, content_type=MIMEType.TYPE_TXT) as response:
        response.send("back")
        robot.motors[MOTOR_LEFT_FRONT].target_throttle = -1 * MOTOR_DEFAULT_THROTTLE
        robot.motors[MOTOR_RIGHT_FRONT].target_throttle = -1 * MOTOR_DEFAULT_THROTTLE
        robot.motors[MOTOR_RIGHT_REAR].target_throttle = -1 * MOTOR_DEFAULT_THROTTLE
        robot.motors[MOTOR_LEFT_REAR].target_throttle = -1 * MOTOR_DEFAULT_THROTTLE

@server.route("/pb/v1/left")
def drive_left(request: HTTPRequest):  # pylint: disable=unused-argument
    #  with content type text/html
    with HTTPResponse(request, content_type=MIMEType.TYPE_TXT) as response:
        response.send("left")
        robot.motors[MOTOR_LEFT_FRONT].target_throttle = -1 * MOTOR_TURNING_THROTTLE
        robot.motors[MOTOR_RIGHT_FRONT].target_throttle = MOTOR_TURNING_THROTTLE
        robot.motors[MOTOR_RIGHT_REAR].target_throttle = MOTOR_TURNING_THROTTLE
        robot.motors[MOTOR_LEFT_REAR].target_throttle = -1 * MOTOR_TURNING_THROTTLE
        
@server.route("/pb/v1/right")
def drive_right(request: HTTPRequest):  # pylint: disable=unused-argument
    #  with content type text/html
    with HTTPResponse(request, content_type=MIMEType.TYPE_TXT) as response:
        response.send("right")
        robot.motors[MOTOR_LEFT_FRONT].target_throttle = MOTOR_TURNING_THROTTLE
        robot.motors[MOTOR_RIGHT_FRONT].target_throttle = -1 * MOTOR_TURNING_THROTTLE
        robot.motors[MOTOR_RIGHT_REAR].target_throttle = -1 * MOTOR_TURNING_THROTTLE
        robot.motors[MOTOR_LEFT_REAR].target_throttle = MOTOR_TURNING_THROTTLE
        
@server.route("/pb/v1/stop")
def drive_stop(request: HTTPRequest):  # pylint: disable=unused-argument
    #  with content type text/html
    with HTTPResponse(request, content_type=MIMEType.TYPE_TXT) as response:
        response.send("stop")
        robot.motors[MOTOR_LEFT_FRONT].target_throttle = 0.0
        robot.motors[MOTOR_RIGHT_FRONT].target_throttle = 0.0
        robot.motors[MOTOR_RIGHT_REAR].target_throttle = 0.0
        robot.motors[MOTOR_LEFT_REAR].target_throttle = 0.0

@server.route("/pb/v1/led-on")
def led_on(request: HTTPRequest):  # pylint: disable=unused-argument
    #  with content type text/html
    global led_api_value
    with HTTPResponse(request, content_type=MIMEType.TYPE_TXT) as response:
        response.send("led on")
    led_api_value = True

@server.route("/pb/v1/led-off")
def led_off(request: HTTPRequest):  # pylint: disable=unused-argument
    #  with content type text/html
    global led_api_value
    with HTTPResponse(request, content_type=MIMEType.TYPE_TXT) as response:
        response.send("led off")
    led_api_value = False
print("starting server..")
# startup the server
try:
    server.start(str(wifi.radio.ipv4_address))
    print("Listening on http://%s:80" % wifi.radio.ipv4_address)
#  if the server fails to begin, restart the pico w
except OSError:
    time.sleep(5)
    print("restarting..")
    microcontroller.reset()
ping_address = ipaddress.ip_address("8.8.4.4")

clock = time.monotonic() - 30 #  time.monotonic() holder for server ping

#target 100hz
target_rate = 0.01 # 1 hundredth of a second for each loop

while True:
    try:
        loop_start = monotonic()
        #  every 30 seconds
        if (clock + 30) < time.monotonic():
            if wifi.radio.ping(ping_address) is None:
                print("lost connection")
            else:
                print("connected")
            clock = time.monotonic()
        #  poll the server for incoming/outgoing requests
        server.poll()
        led.value = led_api_value
        robot.update_throttles()
        loop_duration = (monotonic() - loop_start)/1000
        if loop_duration < target_rate:
            sleep(target_rate - loop_duration)

            
            
        
        
    # pylint: disable=broad-except
    except Exception as e:
        print(e)
        continue
