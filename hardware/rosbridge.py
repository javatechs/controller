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

"""Handle rosbridge Messages"""
#
# Handle rosbridge Messages
#
# See https://github.com/gramaziokohler/roslibpy
#
# Forward command to rosbridge
#
import time
import datetime
import logging
import sys
import threading

import json

import roslibpy
from roslibpy import Ros
from roslibpy import Topic

if (sys.version_info > (3, 0)):
    import urllib.request as urllib2
    from urllib.parse import urlencode
else:
    import urllib2 # pylint: disable=import-error
    from urllib import urlencode

log = logging.getLogger('RemoTV.hardware.rosbridge')

rosbridge = None


def setup(robot_config):
    global rosbridge

    rosbridge = ROSbridge(robot_config)
    return
    
#
def move(args):
    global log
    global rosbridge

    try:
        rosbridge.handle_input(args)
    except Exception as e:
        log.critical("rosbridge Error: " + str(e))

#
#
#
class ROSbridge:
    def __init__(self, robot_config):
        ''' '''
        self.host = 'localhost'
        if robot_config.has_option('rosbridge', 'host'):
            self.host = robot_config.get('rosbridge', 'host')

        self.port = 9000
        if robot_config.has_option('rosbridge', 'port'):
            self.port = robot_config.getint('rosbridge', 'port')

        self.user_slice = 60
        if robot_config.has_option('rosbridge', 'user_slice'):
            self.user_slice = robot_config.getfloat('rosbridge', 'user_slice')

        self.uid = 'admin'
        if robot_config.has_option('rosbridge', 'uid'):
            self.uid = robot_config.get('rosbridge', 'uid')
        self.pwd = ''
        if robot_config.has_option('rosbridge', 'pwd'):
            self.pwd = robot_config.get('rosbridge', 'pwd')

        self.owners = robot_config.get('robot', 'owner').split(',')

        # 
        # Connect to rosbridge
        self.ros = Ros(self.host, self.port)
        # 
        # NOTE; Attempt to connect to rosbridge from init().
        # During handle_input() if not connected, attempt to connect to rosbridge.
        #
        self.pub_remo_cmd_str = Topic(self.ros, '/remo_cmd', 'std_msgs/String')
        self.pub_remo_cmd_str.advertise()
        self.pub_remo_settings = Topic(self.ros, '/remo_settings', 'std_msgs/String',latch=True)
        self.pub_remo_settings.advertise()
        publish_settings(robot_config, self.pub_remo_settings)
        try:
            self.ros.run()
            log.debug("Connected to rosbridge!")
        except Exception as e:
            log.error("Cannot connect to rosbridge!")

        # TODO Add a field based ROS message vs. JSON.

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
#         log.debug("ARGS: "+str(args))
        t = datetime.datetime.utcnow()
        user = args['user']['username']

        if (t - self.last_user_time).total_seconds() > self.user_slice:
            self.prev_command_user = self.last_command_user
            self.last_command_user = None

        if user in self.owners:
            log.debug("rosbridge owner (%s) took over" % user)
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

            log.debug("rosbridge Got command %s from %s" % (command, user))

            # Are we connected?
            if not self.ros.is_connected:
                # No. Retry
                try:
                    self.ros.run()
                    log.debug("Connected to rosbridge!")
                except Exception as e:
                    log.error("Cannot connect to rosbridge!")
            if self.ros.is_connected:
                with self.lock:
                    self.pub_remo_cmd_str.publish(roslibpy.Message({'data': str(args)}))
                
#
# Form JSON string of settings
# Topic data is latched.
#
def publish_settings(robot_config, pub):
    ''' '''
    json_str = {}
    sections = robot_config.sections()
    for section in sections:
        options = robot_config.options(section)
        json_str[section] = {}
        json_elem = {}
        for option in options:
            json_elem[option] = robot_config.get(section, option)
        json_str[section]= json_elem
#     log.error(json.dumps(json_str))
#     print(json.dumps(json_str))
    pub.publish(roslibpy.Message({'data': str(json.dumps(json_str))}))
