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

global FWD, BACK, LEFT, RIGHT, CTRL
      
def on_press(key):
    global FWD, BACK, LEFT, RIGHT, CTRL

    try:
        if key.char == 'w':
            FWD = True
        if key.char == 'a':
            LEFT = True
        if key.char == 's':
            BACK = True
        if key.char == 'd':
            RIGHT = True
        if key.char == 'c' and CTRL == True:
            # Stop keyboard listener.
            return False
    except AttributeError:
        if key == keyboard.Key.ctrl:
            CTRL = True

def on_release(key):
    global FWD, BACK, LEFT, RIGHT, CTRL
    
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
        if key == keyboard.Key.ctrl:
            CTRL = False
        
def send_command(wireless, s):
    global FWD, BACK, LEFT, RIGHT
    
    # Send initial command.
    try:
        s.sendto('x', (UDP_IP, UDP_PORT))
    except socket.error:
        print '\rError sending initial command'
        sys.exit()
    
    while True:
        # Check if still connected to network.
        if wireless.current() == None:
            sys.exit()
    
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
        time.sleep (50.0 / 1000.0);
        
def process_packet(data):
    if data[0] == 'L':
        analog_value = float(data[1:5])
        lux_value = analog_value / 1024.0 * 1000.0;
        return "%.2f lx" % lux_value
    if data[0] == 'U':
        h = int(data[1])
        m = int(data[2:4])
        s = int(data[4:6])
        time = ""
        if h != 0:
            time += "%dh " % (h)
        if m != 0:
            time += "%dm " % (m)
        if s != 0:
            time += "%ds " % (s)
        return time
    
    
    if data[0] == 'E':
        return data[1:]

def update_info(s):
    uptime = '-'
    lux = '-'
    errors = 'None'
    
    
    root = Tk()
    root.title("EXPLORER")
    T = Text(root, height=8, width=30, font=("Helvetica",20), bg="black", fg="white")
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
        #print "\r", addr, data, "\r"
        result = process_packet( data )
        if data[0] == 'L':
            lux = result
        if data[0] == 'U':
            uptime = result
        
        if data[0] == 'E':
            errors = result
        
        T.delete('1.0', END)
        
        T.insert(END, u"Uptime:  %s\n" % ( uptime ))
        T.insert(END, u"Heading:  %s\u00b0 %s'\n" % ('-', '-'))
        T.insert(END, u"Declination:  +5\u00b0 32'\n")
        T.insert(END, u"Temperature:  +%s\u2103\n" % ('-'))
        T.insert(END, u"Pressure:  %s hPa\n" % ('-'))
        T.insert(END, u"Humidity:  %s%s\n" % ('-', '%'))
        T.insert(END, u"Illuminance:  %s\n" % ( lux ))
        T.insert(END, u"Errors:  %s\n" % ('None'))
        
        T.update()
        T.update_idletasks()

def exit_handler():
    curses.flushinp()
    curses.echo()
    curses.endwin()
    print '\rDisconnected.'
    sys.stdout.flush()
    wireless.connect(ssid='ASGARD', password='newjersey')

if __name__ == '__main__':

    global FWD, BACK, LEFT, RIGHT, CTRL

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
    
    FWD = False
    BACK = False
    LEFT = False
    RIGHT = False
    CTRL = False
    
    t1 = threading.Thread(target=send_command, args=[wireless, s] )
    t1.daemon = True
    t1.start()
    
    t2 = threading.Thread(target=update_info, args=[s] )
    t2.daemon = True
    t2.start()
    
    # Start keyboard listener.
    with keyboard.Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        listener.join()
    
    
        
