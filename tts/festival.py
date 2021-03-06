import os
import tempfile
import uuid
import logging

log = logging.getLogger('RemoTV.tts.festival')

tempDir = None
hw_num = None

def setup(robot_config):
    global tempDir
    global hw_num
    
    if robot_config.has_option('tts', 'speaker_num'):
        hw_num = robot_config.getint('tts', 'speaker_num')
    else:
        hw_num = robot_config.getint('tts', 'hw_num')
    
    #set the location to write the temp file to
    tempDir = tempfile.gettempdir()
    log.info("TTS temporary directory : %s", tempDir)

def say(*args):
    message = args[0]

    tempFilePath = os.path.join(tempDir, "text_" + str(uuid.uuid4()))
    f = open(tempFilePath, "w")
    f.write(message)
    f.close()

    os.system('text2wave -o ' + tempFilePath +'.wav ' + tempFilePath)
    os.system('aplay -D plughw:{} '.format(hw_num) + tempFilePath + '.wav')
    os.remove(tempFilePath + '.wav')
    os.remove(tempFilePath)
    
