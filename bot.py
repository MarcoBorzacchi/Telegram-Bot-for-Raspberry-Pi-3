import sys
import os
import random
import time
import serial
import telepot
from telepot.loop import MessageLoop
import RPi.GPIO as GPIO
from picamera import PiCamera
from datetime import datetime
import threading

range_photos = 4
flag_ISR = 0
usb_connected = True
temperature = 0
chosen_temp = 0
changing_t = False
camera = PiCamera()
camera.rotation = 180
private_chat_id = <your_chat_id>

GPIO.setmode(GPIO.BCM)
GPIO.setup(11, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)	# to detect a movement
GPIO.setup(18, GPIO.OUT)								# used as PWM to ring the buzzer
GPIO.setup(27, GPIO.OUT)								# used to switch on/off the LED

""" The serial USB communication try to get establish; if the Access Point is not connected the variable "usb_connected" becomes false """
try:
	ser = serial.Serial(port = '/dev/ttyACM0', baudrate = 9600)
except :
	try:
		ser = serial.Serial(port = '/dev/ttyACM1', baudrate = 9600)
	except:
		print('No serial USB device connected')
		usb_connected = False

# Movement Detection Service Routine
def ISR(channel):
	p = GPIO.PWM(18, 880)
	p.start(50.0)						# start to play the Alarm
	date_time = str(datetime.now())		# save in a string the actual date and time
	path = '/home/pi/Desktop/camera_pictures/' + date_time + '/' 
	os.mkdir(path)			# create a folder with the name "date_time" in the specific path														
	camera.start_preview()				# enable camera
	for i in range(0,range_photos):
		camera.capture(path + str(i)+'.jpg')	# take and save photos in the created folder
		camera.stop_preview()
	for i in range (0, range_photos) :
		bot.sendPhoto(private_chat_id, open(path + str(i) + '.jpg','rb')) # send photos
	p.stop()							# disable camera
	time.sleep(3)

# Acquire Temperature Service Routine
def acquire_t():
	global usb_connected
	global temperature
	global ser
	global chosen_temp
	if usb_connected:			# exploit this ISR only when the Access Point is connected
		x =ser.readline()	# read the serial data (line)
		
		""" Continues to read string lines until these contain the desirable information from End Device (Node:0001). Successively, extract the value of the Temperature placed in a known position (between 'Temp' and 'F,'), convert in Celsius quantity and store in the global variable "temperature". Take care of switching on/off the LED (heater) if the temperature chosen by the user is greater or smaller than the actual temperature. """
		
		while  x.find('Temp') is -1 or x.find('Node:0001') is -1 :
			time.sleep(0.1)
			x = ser.readline()
		y = x.find('Temp')
		z = x.find('F,')
		temperature = (float(x[y + 6: z ]) - 32)/ 1.8
		if chosen_temp > temperature :
			GPIO.output(27, GPIO.HIGH)
		else :
			GPIO.output(27, GPIO.LOW)
		threading.Timer(2, acquire_t).start()	# call the ISR each 2 seconds

# Method that is called each time the user sends a message. It checks if the incoming message corresponds to something notable and it will react accordingly.
def handle(msg):
	global private_chat_id
	global changing_t
	global chosen_temp
	content_type, chat_type, chat_id= telepot.glance(msg)
	print(content_type, chat_type, private_chat_id)
	if chat_id == private_chat_id :		# chat established only with the specified user
		if changing_t :
			try:
				chosen_temp = int(msg[content_type])
				if (chosen_temp <= 35 and chosen_temp >= 0) :
					if chosen_temp > temperature :
						bot.sendMessage(chat_id, 'The Temperature has been set to ' +  msg[content_type] +  ' C degree')
						GPIO.output(27, GPIO.HIGH)
					else :
						bot.sendMessage(chat_id, 'The Temperature is already greater than  ' +  msg[content_type] +  ' C degree')
						GPIO.output(27, GPIO.LOW)
						changing_t = False
				else :
					bot.sendMessage(chat_id, 'Choose a Temperature between 0 and 35 C degrees')
			except:
				bot.sendMessage(chat_id, 'Insert a number between 0 and 35 to choose the Temperature')
		
		elif msg[content_type] in ['Photo', 'photo'] :
			date_time = str(datetime.now())
			path = '/home/pi/Desktop/camera_pictures/requested_photos/'
			camera.start_preview()
			try :	
				camera.capture(path + date_time + '.jpg')
			except:		# if it get an error create the folder "requested_photos" first
				os.mkdir(path)
				camera.capture(path + date_time + '.jpg')
			camera.stop_preview()
			bot.sendPhoto(chat_id, open(path + date_time + '.jpg'))
		
		elif msg[content_type] in['enable_alarm', 'Enable_alarm', 'enable alarm', 'Enable alarm'] :
			bot.sendMessage(chat_id, 'Alarm has been activated')
			GPIO.add_event_detect(11, GPIO.RISING, callback=ISR, bouncetime=300)
		
		elif msg[content_type] in ['disable_alarm', 'Disable_alarm', 'disable alarm', 'Disable alarm'] :
			bot.sendMessage(chat_id, 'Alarm has been deactivated')
			GPIO.remove_event_detect(11)
		
		elif msg[content_type] in ['temperature', 'Temperature', 'T', 't']:
			global temperature
			bot.sendMessage(chat_id, 'The actual temperature is ' + str(round(temperature, 2)) + ' C degrees')
			
		elif msg[content_type] in ['change t', 'Change T', 'Change t', 'change T']:
		changing_t = True
			bot.sendMessage(chat_id, 'Choose a Temperature between 0 and 35 C degrees ')


try :

	# Create the object bot from library telepot, class Bot indicating the specific
	# token, given by the "BotFather" during the Bot creation
	bot = telepot.Bot('<your_Bot_token>') 
	
	# MessageLoop is a method imported from telepot.loop (library telepot, clas loop). 
	# It takes as parameter the object bot and the function handle.
	# The method run_as_thread (defined in MessageLoop) takes the object described
	# above in order to apply the Telegram threading functions
	MessageLoop(bot, handle).run_as_thread()
	
	acquire_t()	# Enable the Timer ISR to get the temperature each 2 seconds
	print ('Listening ...')
	# Keep the program running.
	while 1:
		time.sleep(10)  # these sleep allows the various threads switching
		
except KeyboardInterrupt:
	GPIO.cleanup()
