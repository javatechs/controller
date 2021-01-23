#!/usr/bin/env python
#
# Copyright 2021 by Mendocino Coast Makers Guild.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

"""Handle Amcrest Messages"""
#
# Handle Amcrest Messages
# 
# For camera information see http://amcrest.com/
# Uses library from https://github.com/tchellomello/python-amcrest/
#
# Forward command to web controller
#
import time
import datetime
# import json
import logging
import sys
import threading

from amcrest import AmcrestCamera

if (sys.version_info > (3, 0)):
    import urllib.request as urllib2
    from urllib.parse import urlencode
else:
    import urllib2 # pylint: disable=import-error
    from urllib import urlencode

log = logging.getLogger('RemoTV.hardware.amcrest')

amcrest = None


def setup(robot_config):
    global amcrest

    amcrest = Amcrest(robot_config)
    return
    
#
def move(args):
    global log
    global amcrest

    try:
        amcrest.handle_input(args)
    except Exception as e:
        log.critical("Amcrest Error: " + str(e))

#
#
#
class Amcrest:
    def __init__(self, robot_config):
        ''' '''
        self.host = 'localhost'
        if robot_config.has_option('amcrest', 'host'):
            self.host = robot_config.get('amcrest', 'host')

        self.port = 9000
        if robot_config.has_option('amcrest', 'port'):
            self.port = robot_config.getint('amcrest', 'port')

        self.user_slice = 60.0
        if robot_config.has_option('amcrest', 'user_slice'):
            self.user_slice = robot_config.getfloat('amcrest', 'user_slice')

        self.uid = 'admin'
        if robot_config.has_option('amcrest', 'uid'):
            self.uid = robot_config.get('amcrest', 'uid')
        self.pwd = ''
        if robot_config.has_option('amcrest', 'pwd'):
            self.pwd = robot_config.get('amcrest', 'pwd')

        self.owners = robot_config.get('robot', 'owner').split(',')

        # 
        log.warn("111")
        self.camera = AmcrestCamera(self.host, self.port, self.uid, self.pwd).camera
        log.warn("222")

        # Time etc.
        self.last_command_time = datetime.datetime.utcnow()
        self.last_command_user = None
        self.prev_command_user = None
        self.last_user_time = datetime.datetime.utcnow()

        self.lock = threading.Lock()

#
#
#
    def handle_input(self, args):
        ''' Handle input from remo'''
        #
#         if self.conn is None:
#             self.conn = http.client.HTTPConnection(self.host, self.port)
        log.debug("ARGS: "+str(args))
        t = datetime.datetime.utcnow()
        user = args['user']['username']

        if (t - self.last_user_time).total_seconds() > self.user_slice:
            self.prev_command_user = self.last_command_user
            self.last_command_user = None

        if user in self.owners:
            log.debug("Amcrest owner (%s) took over" % user)
            self.prev_command_user = None
            self.last_command_user = user
            self.last_user_time = t

        if self.last_command_user == None:
            if user == self.prev_command_user:
                if (t - self.last_user_time).total_seconds() <= self.user_slice / 3:
                    return
            self.last_command_user = user
            self.last_user_time = t

        if self.last_command_user == user:
            command = args['button']['command']
            commandarray = command.split(",")

            usr = args['user']['username']

            log.debug("Amcrest Got command %s from %s" % (command, user))

            with self.lock:
                if command == 'ptz.zoom,in':
                    try:
                        self.camera.zoom_in("start")
                        time.sleep(.5)
                        self.camera.zoom_in("stop")
                    except Exception as e:
                        log.error("An exception occurred during zoom in\n", exc_info=True)
                #
                elif command == 'ptz.zoom,out':
                    try:
                        self.camera.zoom_out("start")
                        time.sleep(.5)
                        self.camera.zoom_out("stop")
                    except Exception as e:
                        log.error("An exception occurred during zoom out\n", exc_info=True)
                #
                elif commandarray[0] == 'ptz.start':
                    self.pan(commandarray[1], user)
                #

    def pan(self, direction, user):
        """Pan tilt zoom"""
#        if user not in self.owners:
#            return
        try:
            self.camera.ptz_control_command(action="start", code=direction, arg1=0, arg2=2, arg3=0)
            time.sleep(1.0)
            self.camera.ptz_control_command(action="stop", code=direction, arg1=0, arg2=2, arg3=0)
        except Exception as e:
            log.error("An exception occurred in pan("+direction+")\n", exc_info=True)


