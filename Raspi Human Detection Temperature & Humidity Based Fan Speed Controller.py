import math
import time
import board
import adafruit_dht
import RPi.GPIO as GPIO          
from time import sleep
from imutils import face_utils
import dlib
import cv2
from imutils.video import VideoStream
import imutils
import time
import RPi.GPIO as GPIO
import paho.mqtt.client as paho

#Variable definition
# Sensors

#Initial the dht device, with data pin connected to:

#DHT11 Sensors
temperature_c1 = 0
temperature_c2 = 0
humidity1 = 0
humidity2 = 0

dhtDevice1 = adafruit_dht.DHT11(board.D4)
dhtDevice2 = adafruit_dht.DHT11(board.D23)

#L298N motor controller pin definition
in1 = 24
in2 = 25
in3 = 5
in4 = 6
en = 18
eb = 12
temp1=1

#DC fan 
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)


GPIO.setup(en,GPIO.OUT)
GPIO.setup(eb,GPIO.OUT)

GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)

p1=GPIO.PWM(en,1000)
p2=GPIO.PWM(eb,1000)
p1.start(40)
p2.start(40)

#Fan Speed Calculation
def SpeedCalculator(n,t,h):
    if(n == 0):
        speed= 0
        
    elif(n > 0):
        x = 40
        speed=(x + ((n*(t-22)*(h-40) )/20))

    return speed

#Fan Rotation for DC fan1
def FanIsRotating1(speed):
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    p1.ChangeDutyCycle(speed)
    print("Fan1 is running...")

    
#Fan Rotation for DC fan2    
def FanIsRotating2(speed):
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
    p2.ChangeDutyCycle(speed)
    print("Fan2 is running...")
    print("-------------------------------------------------------")

#Fan Off
def FanOFF(speed):
    GPIO.output(in1,GPIO.Low)
    GPIO.output(in2,GPIO.LOW)

############################################################################################################################

##............Publishing def here................. 4 methods 
##  def temp change
    
def ParameterConcat(Temperature1 , Humidity1, Temperature2, Humidity2, HumanCount, Speed1, Speed2):
    
        # Convert both the parameters to String        
        Temperature1 = str(Temperature1)
        Humidity1 = str(Humidity1)
        Temperature2 = str(Temperature2)
        Humidity2 = str(Humidity2)
        HumanCount = str(HumanCount)
        Speed1 = str(Speed1)
        Speed2 = str(Speed2)
        
        # Concatenate the strings
        Parameters = Temperature1 + ','+Humidity1 + ','+ Temperature2 + ','+ Humidity2 + ','+ HumanCount + ','+ Speed1 + ','+ Speed2   
        return str(Parameters)

#Publish values MQTT server
def tempChange(value1):
    
    client = paho.Client("client-035")  # creating an MQTT client
    # broker Address
    broker_address = "ultimate-farmer.cloudmqtt.com"
    # broker port
    port = 1883
    # connection username
    user = "yldjvgmq"
    # connection password
    password = "mRMfj1DwZuUD"

    # create new instance
    # set username and password
    client.username_pw_set(user, password=password)
    print("connecting to broker")
    # connect
    client.connect(broker_address, port=port)
    print("publishing")
    print("-------------------------------------------------------")
    # publish
    client.publish("temp1/RB", value1)
    # client.publish("hum1/RB", value2)
    # time.sleep(4)
    # disconnecting after publishing
    # disconnect
    client.disconnect()

##############################################################################################################################

#Detect Human count
detector = dlib.get_frontal_face_detector()

print("Starting Face Detection")
c = VideoStream(usePiCamera=True).start()       #For Raspberry Pi Camera module
time.sleep(2.0)

human_count=0

while True: 
    #Detect Human count
    frame = c.read()
    frame = imutils.resize(frame, width=450)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces_detected = detector(gray, 0)
    rects = detector(gray, 0)

    for rect in rects:
        x1 = rect.left()
        y1 = rect.top()
        x2 = rect.right()
        y2 = rect.bottom()
        frame = cv2.rectangle(frame, (x1,y1), (x2,y2), (255, 0, 0), 2)
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    
    human_count=len(faces_detected)
    print('Human count inside the room :',human_count)
        
    try:

        # Print the values to the serial port
         temperature_c1 = dhtDevice1.temperature
         temperature_c2 = dhtDevice2.temperature
         humidity1 = dhtDevice1.humidity
         humidity2 = dhtDevice2.humidity
       
         temperature_f1 = temperature_c1 * (9 / 5) + 32
         temperature_f2 = temperature_c2 * (9 / 5) + 32
         
         print("-------------------------------------------------------")
         
         print("Sensor1 Temp: {:.1f} F / {:.1f} C   Sensor1  Humidity: {}% ".format(temperature_f1, temperature_c1, humidity1))
         print("Sensor2 Temp: {:.1f} F / {:.1f} C   Sensor2 Humidity: {}% ".format(temperature_f2, temperature_c2, humidity2))

         print("-------------------------------------------------------")
                  
        
    except RuntimeError as error:  
     time.sleep(2.0)
    
    #Calling SpeedCalculation Function
    speed1=SpeedCalculator(human_count,temperature_c1,humidity1)
    speed2=SpeedCalculator(human_count,temperature_c2,humidity2)    
    print('DC 1 Fan Speed: {}%   DC 2 Fan Speed: {}%'.format(speed1, speed2))
    print("-------------------------------------------------------")
    
    #Calling ParameterConcatinate Function
    CombinedParameter = ParameterConcat(temperature_c1,humidity1,temperature_c2,humidity2,human_count,speed1,speed2)
    print(CombinedParameter)
    #Calling publish Function
    tempChange(CombinedParameter)
    
    # Change Fan Speed
    FanIsRotating1(speed1)
    FanIsRotating2(speed2)
    
