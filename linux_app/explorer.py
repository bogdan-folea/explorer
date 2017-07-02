# -*- coding: utf-8 -*-#########################################################
#                                                                              #
#   Proiect de Licenta                                                         #
#   Bogdan Folea, 2017                                                         #
#                                                                              #
#   explorer.py                                                                #
#                                                                              #
################################################################################


import socket, sys, time, os
import atexit, curses, getch
import threading, Tkinter

from Tkinter import *
from wireless import Wireless
from pynput import keyboard

UDP_IP = "12.0.0.1"
UDP_PORT = 1032

global FWD, BACK, LEFT, RIGHT
      
def on_press(key):
    global FWD, BACK, LEFT, RIGHT

    try:
        if key.char == 'w':
            FWD = True
        if key.char == 'a':
            LEFT = True
        if key.char == 's':
            BACK = True
        if key.char == 'd':
            RIGHT = True
    except AttributeError:
        # Stop keyboard listener
        if key == keyboard.Key.ctrl:
            return False

def on_release(key):
    global FWD, BACK, LEFT, RIGHT
    
    try:
        if key.char == 'w':
            FWD = False
        if key.char == 'a':
            LEFT = False
        if key.char == 's':
            BACK = False
        if key.char == 'd':
            RIGHT = False
    except AttributeError:
        # Stop keyboard listener
        if key == keyboard.Key.ctrl:
            return False
        
def send_command(s):
    global FWD, BACK, LEFT, RIGHT
    
    while True:
        cmd = 'x'
        
        if FWD == True:
            cmd = 'f'
        else:
            if BACK == True:
                cmd = 'b'
            else:
                if LEFT == True:
                    cmd = 'l'
                else:
                    if RIGHT == True:
                        cmd = 'r'
        
        if cmd != 'x':
            #print "\r command: ", cmd
            try:
                s.sendto(cmd, (UDP_IP, UDP_PORT))
            except socket.error, cmd:
                print '\rError sending ' + cmd
                sys.exit()
        time.sleep (100.0 / 1000.0);

def update_info():
    root = Tk()
    T = Text(root, height=10, width=30, font=("Helvetica",20), bg="black", fg="white")
    T.pack()
    T.insert(END, u"Uptime: %s\n" % ('-'))
    T.insert(END, u"Heading: %s\u00b0 %s'\n" % ('-', '-'))
    T.insert(END, u"Declination: +5\u00b0 32'\n")
    T.insert(END, u"Temperature: +%s\u2103\n" % ('-'))
    T.insert(END, u"Pressure: %s hPa\n" % ('-'))
    T.insert(END, u"Humidity: %s%s\n" % ('-', '%'))
    T.insert(END, u"Illuminance: %s lux\n" % ('-'))
    T.insert(END, u"Errors: %s\n" % ('None'))
    T.update()
    T.update_idletasks()
    
    while True:
        data, addr = s.recvfrom(1024)
        print "\r", addr, data, "\r"
        
        T.delete('1.0', END)
        
        T.insert(END, u"Uptime: %s\n" % ('-'))
        T.insert(END, u"Heading: %s\u00b0 %s'\n" % ('-', '-'))
        T.insert(END, u"Declination: +5\u00b0 32'\n")
        T.insert(END, u"Temperature: +%s\u2103\n" % ('-'))
        T.insert(END, u"Pressure: %s hPa\n" % ('-'))
        T.insert(END, u"Humidity: %s%s\n" % ('-', '%'))
        T.insert(END, u"Illuminance: %s lux\n" % ('-'))
        T.insert(END, u"Errors: %s\n" % ('None'))
        
        T.update()
        T.update_idletasks()

def exit_handler():
    curses.echo()
    curses.endwin()
    #os.system('xset r rate 500 33')
    print '\rDisconnected.'
    wireless.connect(ssid='ASGARD', password='newjersey')

if __name__ == '__main__':

    print "\rConnecting to 'EXPLORER'...",
    sys.stdout.flush()
    wireless = Wireless()
    ret = wireless.connect(ssid='EXPLORER', password='asgard')
    if ret == False:
        print "failed"
        sys,exit()
    print "done"

    s = None
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except socket.error:
        print '\rFailed to create socket'
        sys.exit()
    
    curses.initscr()
    curses.noecho()
    
    sys.stderr = open(os.devnull, 'w')
    atexit.register(exit_handler)
    #os.system('xset r rate 20 100')
    
    FWD = False
    BACK = False
    LEFT = False
    RIGHT = False
    
    t1 = threading.Thread(target=send_command, args=[s] )
    t1.daemon = True
    t1.start()
    
    t2 = threading.Thread(target=update_info, args=[] )
    t2.daemon = True
    t2.start()
    
    # Start keyboard listener
    with keyboard.Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        listener.join()
    
    
        
