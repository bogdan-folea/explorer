from pynput import keyboard
import time, sys

global FWD, BACK, LEFT, RIGHT

def on_press(key):
    global FWD, BACK, LEFT, RIGHT

    try:
        print key.char, "pressed"
        if key.char == 'w':
            print "OK"
            FWD = True
        if key.char == 'a':
            LEFT = True
        if key.char == 's':
            BACK = True
        if key.char == 'd':
            RIGHT = True
    except AttributeError:
        # Stop listener
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
        # Stop listener
        if key == keyboard.Key.ctrl:
            return False


    
if __name__ == '__main__':
    global FWD, BACK, LEFT, RIGHT
    
    FWD = False
    BACK = False
    LEFT = False
    RIGHT = False
    
    # Start keyboard listener
    with keyboard.Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        listener.join()
    
    while True:
        print FWD, BACK, LEFT, RIGHT
        sys.stdout.flush()
        '''
        if FWD == True:
            print "FORWARD"
        if BACK == True:
            print "BACK"
        if LEFT == True:
            print "LEFT"
        if RIGHT == True:
            print "RIGHT"     
        
        #time.sleep( 1 )
        '''

