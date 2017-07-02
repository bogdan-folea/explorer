from pynput import keyboard
import threading, time


def send_command():
    
    while True:
        print "ceva"
        time.sleep (1000.0 / 1000.0);


def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(
            key.char))
    except AttributeError:
        print('special key {0} pressed'.format(
            key))

def on_release(key):
    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False


    
if __name__ == '__main__':
    
    t = threading.Thread(target=send_command, args=[] )
    t.daemon = True
    t.start()
    
    # Collect events until released
    with keyboard.Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        listener.join()
