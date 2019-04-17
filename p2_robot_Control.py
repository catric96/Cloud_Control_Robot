from pynput.keyboard import Key, Listener


#speed = input("Input speed of travel desired (0-255): ")
#theta = input("Input direction of turn (-180 - 0 - +180)")
class controlStruct():
    def __init__(velocity, theta, mode):
        self.velocity = velocity
        self.theta = theta
        self.mode = mode

def on_press(key):
	print('{0} pressed'.format(key))

def on_release(key):
	print(key)
	if key == 'w':
		print('Robot increase speed')
	if key == 's':
		print('Robot decrease speed')
	if key == 'u'a'':
		print('Robot go left')
	if key == Key.right:
		print('Robot go right')
	if(key == Key.esc):
		return False

with Listener( on_press=on_press, on_release=on_release) as listener:
	listener.join()