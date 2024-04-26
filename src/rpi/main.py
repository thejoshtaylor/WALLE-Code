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
START_FILE = "tests/test-tank-drive-talon-and-actuator.py"

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
# add handlers
log.addHandler(handler)
log.addHandler(consoleHandler)

# Main process variable
mainProcess = None

# Functions
def update():
    # Check for internet connection
    try:
        socket.create_connection(("www.google.com", 80))
    except OSError:
        log.info("No internet connection")
        return
    
    # Check for git remote updates
    log.info("Checking for updates")
    try:
        # run git command and get output
        log.debug("Running `git remote -v update`")
        output = subprocess.check_output("git remote update", shell=True)
        log.debug(output)
        # check if there are updates
        log.debug("Running `git status`")
        output = subprocess.check_output("git status", shell=True)
        log.debug(output)
        if "Your branch is behind" in output:
            log.info("Updates available")
            # Turn on the RPI LED
            log.debug("Turning on LED")
            output = subprocess.check_output("echo 1 | sudo tee /sys/class/leds/ACT/brightness", shell=True)
            log.debug(output)
            # pull updates
            log.debug("Running `git pull`")
            output = subprocess.check_output("git pull", shell=True)
            log.debug(output)
            # restart program
            restart()

    except:
        log.error("Error checking for updates")

def start():
    global mainProcess
    if mainProcess is not None:
        log.info("Program already running")
        return
    
    log.info("Starting program")
    try:
        # Start the main program as a different process but keep track of it
        log.debug("Starting program")
        mainProcess = subprocess.Popen(["python3", START_FILE])
        log.debug("Started program")
    except:
        log.error("Error starting program")
    
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
            log.error("Main program crashed")
            # Restart the main program
            start()

        # Sleep
        time.sleep(10)

if __name__ == "__main__":
    # Main loop
    try:
        main()
    except KeyboardInterrupt:
        log.info("Keyboard interrupt")
        stop()
        sys.exit(0)
    except:
        log.error("Main program crashed")
        stop()
        sys.exit(1)