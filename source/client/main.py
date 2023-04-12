import board
from digitalio import DigitalInOut, Direction, Pull
from time import monotonic, sleep
import wifi
import socketpool
import ipaddress
import os
import adafruit_requests

btn_left = DigitalInOut(board.GP16)
btn_up= DigitalInOut(board.GP17)
btn_right = DigitalInOut(board.GP18)
btn_down = DigitalInOut(board.GP15)

btn_left.direction = Direction.INPUT
btn_up.direction = Direction.INPUT
btn_right.direction = Direction.INPUT
btn_down.direction = Direction.INPUT

btn_left.pull = Pull.UP
btn_up.pull = Pull.UP
btn_right.pull = Pull.UP
btn_down.pull = Pull.UP


#  set static IP address
client_ipv4 =  ipaddress.IPv4Address(os.getenv('REST_ROVER_CLIENT_IPV4'))

netmask =  ipaddress.IPv4Address(os.getenv('CIRCUITPY_NETMASK'))
gateway =  ipaddress.IPv4Address(os.getenv('CIRCUITPY_GATEWAY'))
wifi.radio.set_ipv4_address(ipv4=client_ipv4,netmask=netmask,gateway=gateway)
#  connect to your SSID
wifi.radio.connect(os.getenv('CIRCUITPY_WIFI_SSID'), os.getenv('CIRCUITPY_WIFI_PASSWORD'))

print("Connected to WiFi")
pool = socketpool.SocketPool(wifi.radio)

REST_ROVER_PROTOCOL = os.getenv('REST_ROVER_PROTOCOL')
if REST_ROVER_PROTOCOL == 'http':
    rate = 0.33 #33hz

    server_uri = 'http://' + os.getenv('REST_ROVER_SERVER_IPV4') + '/pb/v1/'

    requests = adafruit_requests.Session(pool)

    while True:
        loop_start = monotonic() * 1000
        
        api_call = 'stop'
        
        #buttons are low when activated so we check for false
        if not btn_left.value:
            api_call = 'led_on'
        elif not btn_up.value:
            api_call = 'led_off'
        elif not btn_right.value:
            api_call = 'led_on'
        elif not btn_down.value:
            api_call = 'led_off'

        try:
            response = requests.get(server_uri+ api_call)
            print(response.text)
        except Exception as e:
            print(str(e))

        loop_durnation = (monotonic() * 1000) - loop_start
        if loop_durnation < rate:
            sleep(rate-loop_durnation)
elif REST_ROVER_PROTOCOL == 'udp':
    ## from https://github.com/anecdata/Socket/blob/main/examples/udp_client_CircuitPython_NATIVE.py
    
    HOST = os.getenv('REST_ROVER_SERVER_IPV4')
    PORT = 5000
    TIMEOUT = 5
    INTERVAL = 0.1
    MAXBUF = 256

    print("Self IP", wifi.radio.ipv4_address)
    server_ipv4 = ipaddress.ip_address(pool.getaddrinfo(HOST, PORT)[0][4][0])
    print("Server ping", server_ipv4, wifi.radio.ping(server_ipv4), "ms")

    buf = bytearray(MAXBUF)
    print("Create UDP Client socket")
    s = pool.socket(pool.AF_INET, pool.SOCK_DGRAM)
    s.settimeout(TIMEOUT)
    while True:
        #buttons are low when activated so we check for false
        api_call = 'stop'
        if not btn_left.value:
            api_call = 'left'
        elif not btn_up.value:
            api_call = 'forward'
        elif not btn_right.value:
            api_call = 'right'
        elif not btn_down.value:
            api_call = 'back'


        size = s.sendto(api_call, (HOST, PORT))
        print("Sent", size, "bytes")

        #size, addr = s.recvfrom_into(buf)
        #print("Received", buf[:size], size, "bytes from", addr)
        sleep(INTERVAL)
     
    s.close()
    # while True:
    #             #buttons are low when activated so we check for false
    #     api_call = 'stop'
    #     if not btn_left.value:
    #         api_call = 'led_on'
    #     elif not btn_up.value:
    #         api_call = 'led_off'
    #     elif not btn_right.value:
    #         api_call = 'led_on'
    #     elif not btn_down.value:
    #         api_call = 'led_off'



    #     size = s.sendto(b"Hello, world", (HOST, PORT))
    #     print("Sent", size, "bytes")

    #     # size, addr = s.recvfrom_into(buf)
    #     # print("Received", buf[:size], size, "bytes from", addr)

        

    #     sleep(INTERVAL)
    # s.close()



