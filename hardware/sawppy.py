import datetime
import json
import logging
import sys
import threading
import time

if (sys.version_info > (3, 0)):
    import urllib.request as urllib2
    from urllib.parse import urlencode
else:
    import urllib2 # pylint: disable=import-error
    from urllib import urlencode

log = logging.getLogger('RemoTV.hardware.sawppy')

sawppy = None

def setup(robot_config):
    global sawppy

    sawppy = Sawppy(robot_config)

def move(args):
    global log
    global sawppy

    try:
        sawppy.handle_input(args)
    except Exception as e:
        log.critical("Sawppy Error: " + str(e))

class Sawppy:
    def __init__(self, robot_config):
        self.motor_time = 0.5
        if robot_config.has_option('sawppy', 'motor_time'):
            self.motor_time = robot_config.getfloat('sawppy', 'motor_time')
        self.driving_speed = 0.6
        if robot_config.has_option('sawppy', 'driving_speed'):
            self.driving_speed = robot_config.getfloat('sawppy', 'driving_speed')
        self.turn_angle = 0.4
        if robot_config.has_option('sawppy', 'turn_angle'):
            self.turn_angle = robot_config.getfloat('sawppy', 'turn_angle')
        self.user_slice = 60
        if robot_config.has_option('sawppy', 'user_slice'):
            self.user_slice = robot_config.getfloat('sawppy', 'user_slice')
        log.critical("Sawppy: speed= %f  angle= %f  motor_time= %f  user_slice= %f" % (self.driving_speed, self.turn_angle, self.motor_time, self.user_slice))

        self.last_command_time = datetime.datetime.utcnow()
        self.last_command_user = None
        self.last_user_time = datetime.datetime.utcnow()
        self.stopped = False
        self.lock = threading.Lock()
        self.magnitude = 0.0
        self.angle = 0.0

        # Start the main_loop() in a separate thread
        self.tsk = threading.Thread(target = self.main_loop)
        self.tsk.daemon = True
        self.tsk.start()
        log.debug("Sawppy Activated")


    def handle_input(self, args):
        t = datetime.datetime.utcnow()
        user = args['user']['username']

        if (t - self.last_user_time).total_seconds > self.user_slice:
            self.last_command_user = None

        if self.last_command_user == None:
            self.last_command_user = user
            self.last_user_time = t

        if self.last_command_user == user:
            command = args['button']['command']

            log.debug("Sawppy Got command %s from %s" % (command, user))

            with self.lock:
                if command == 'forward':
                    self.magnitude = self.driving_speed
                    self.angle = 0.0
                elif command == 'forwardleft':
                    self.magnitude = self.driving_speed
                    self.angle = -self.turn_angle
                elif command == 'forwardright':
                    self.magnitude = self.driving_speed
                    self.angle = self.turn_angle
                elif command == 'left':
                    self.magnitude = 0.15
                    self.angle = -1.0
                elif command == 'right':
                    self.magnitude = 0.15
                    self.angle = 1.0
                elif command == 'reverse':
                    self.magnitude = -self.driving_speed
                    self.angle = 0.0
                elif command == 'reverseleft':
                    self.magnitude = -self.driving_speed
                    self.angle = -self.turn_angle
                elif command == 'reverseright':
                    self.magnitude = -self.driving_speed
                    self.angle = self.turn_angle
                elif command == 'stop':
                    self.magnitude = 0.0
                    self.angle = 0.0
                self.last_command_time = datetime.datetime.utcnow()

    def main_loop(self):
        self.running = True
        log.debug("Sawppy Loop starting!")
        send_last_command = datetime.datetime.utcnow()
        try:
            while self.running:
                with self.lock:
                    lc = self.last_command_time
                    m = self.magnitude
                    a = self.angle
                    s = self.stopped

                t = datetime.datetime.utcnow()
                # log.critical("main_loop send_last_command= %s  lc = %s (%f), m= %f, a= %f, s= %d, t= %s" % (str(send_last_command), str(lc), (send_last_command - lc).total_seconds(), m, a, s, str(t)))

                if (send_last_command - lc).total_seconds() < 0.0:
                    # log.critical("main_loop ===> m= %f, a= %f" % (m, a))
                    self.drive(m, a)
                    send_last_command = t
                elif (t - send_last_command).total_seconds() > self.motor_time and not s:
                    # log.critical("main_loop timeout !! stop")
                    self.drive(0, 0)
                if self.last_command_user != None and (t - self.last_user_time).total_seconds() > self.user_slice:
                    # log.critical("user switch")
                    self.last_command_user = None

                time.sleep(0.05)
        except Exception as e:
            print("Sawppy Loop Error: " + str(e))
            log.critical("Sawppy Loop Error: " + str(e))

    def drive(self, speed, angle):
        data = urlencode({ 'magnitude': int(speed * 100), 'pct_angle': int(angle * 100)})
        # log.critical("mw data= " + data)
        req = urllib2.Request('http://127.0.0.1:5000/drive_command', data = data)
        try:
            f = urllib2.urlopen(req)
        except Exception as e:
            print("Sawppy drive Error: " + str(e))

        with self.lock:
            if speed == 0.0:
                self.stopped = True
            else:
                self.stopped = False
