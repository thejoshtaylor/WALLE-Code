# WALL-E
# UCLA ASME X1 Robotics Club
# Joshua Taylor
# 2023-24

# Description: Main program for WALL-E operation
# This program will run on startup on the raspberry pi.
# It makes sure that the main loops run properly and
# handles crashes/errors.
# It will check for a connection to the internet for
# over the air updates from the GitHub repository
# and automatically update and restart the program
# if a new version is available.

# Import
import os
import sys
import time
import subprocess
import logging
import logging.handlers
import socket
from datetime import datetime

# Constants
VERSION = "0.0.1"
START_FILE = "run-system.py"

# Logging
if not os.path.exists("logs"):
    os.makedirs("logs")
# create logger
log = logging.getLogger("main")
log.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
# log to rotating files
handler = logging.handlers.RotatingFileHandler("logs/main.log", maxBytes=1000000, backupCount=5)
handler.setFormatter(formatter)
# log to stdout
consoleHandler = logging.StreamHandler()
consoleHandler.setFormatter(formatter)
consoleHandler.setLevel(logging.INFO)
# add handlers
log.addHandler(handler)
log.addHandler(consoleHandler)

# Main process variable
mainProcess = None

# Functions
def update():
    # Check for internet connection
    try:
        log.debug("Checking for internet connection")
        socket.create_connection(("www.google.com", 80))
    except OSError:
        log.info("No internet connection")
        return
    
    # Check for git remote updates
    log.info("Checking for updates")
    try:
        # run git command and get output
        log.debug("Running `git remote -v update`")
        output = subprocess.check_output("git remote update", shell=True, cwd=os.path.dirname(os.path.realpath(__file__)) + "/", stderr=subprocess.STDOUT)
        log.debug(output)
        # check if there are updates
        log.debug("Running `git status`")
        output = subprocess.check_output("git status", shell=True, cwd=os.path.dirname(os.path.realpath(__file__)) + "/", stderr=subprocess.STDOUT)
        log.debug(output)
        if b"Your branch is behind" in output:
            log.info("Updates available")
            # Turn on the RPI LED
            log.debug("Turning on LED")
            output = subprocess.check_output("echo 1 | sudo tee /sys/class/leds/ACT/brightness", shell=True, stderr=subprocess.STDOUT)
            log.debug(output)
            # pull updates
            log.debug("Running `git pull`")
            output = subprocess.check_output("git pull", shell=True, cwd=os.path.dirname(os.path.realpath(__file__)) + "/", stderr=subprocess.STDOUT)
            log.debug(output)
            # restart program
            restart()
        else:
            log.info("No updates available")

    except Exception as e:
        log.error("Error checking for updates")
        log.error(e)

def start():
    global mainProcess
    if mainProcess is not None and mainProcess.poll() is None:
        log.info("Program already running")
        return
    
    log.info("Starting program")
    log.debug("cwd: " + os.getcwd())
    log.debug("pwd: " + os.path.dirname(os.path.realpath(__file__)) + "/")
    log.debug("cmd: python3 " + START_FILE)
    try:
        # Start the main program as a different process but keep track of it
        subLog = open("logs/main-sub.log", "w")
        mainProcess = subprocess.Popen(["python3", START_FILE], cwd=os.path.dirname(os.path.realpath(__file__)) + "/", stdout=subLog, stderr=subprocess.STDOUT)
        log.debug("Started program")
    except Exception as e:
        log.error("Error starting program")
        log.error(e)
    
def stop():
    global mainProcess
    if mainProcess is None:
        log.info("Program not running")
        return
    
    log.info("Stopping program")
    try:
        # Stop the main program
        log.debug("Stopping program")
        mainProcess.terminate()
        log.debug("Stopped program")
    except:
        log.error("Error stopping program")

def restart():
    log.info("Restarting program")

    # Stop the main program
    stop()

    # Restart this program
    os.execl(sys.executable, sys.executable, *sys.argv)

def main():
    # Turn off the RPI LED
    log.debug("Turning off LED")
    output = subprocess.check_output("echo 0 | sudo tee /sys/class/leds/ACT/brightness", shell=True)
    log.debug(output)

    # Start the main program
    global mainProcess
    start()

    # Main loop
    while True:
        # Check for updates
        update()

        # Check if the main program is still running
        if mainProcess.poll() is not None:
            log.error("Program crashed with error code: " + str(mainProcess.poll()))
            # Restart the main program
            start()

        # Sleep
        time.sleep(10)

if __name__ == "__main__":
    # Main loop
    try:
        main()
    except KeyboardInterrupt as e:
        log.info("Keyboard interrupt")
        stop()
        sys.exit(0)
    except Exception as e:
        log.fatal("Main program crashed")
        log.error(e)
        stop()
        sys.exit(1)