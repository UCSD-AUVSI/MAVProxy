import socket, threading
import server_multiport


slistener = server_multiport.server()
slistener_msgs = []


def send(msg):
	# This port number needs to be the same as expected by the MissionDirector
	#
	PORTNUM = 9982
	thread = threading.Thread(target=private___dispatch_msg, args=(msg,PORTNUM))
	thread.daemon = True
	thread.start()


def check_for_msg():
	
	global slistener
	if slistener.readytostart:
		ports_and_callbacks = []
		
		# This port number needs to be the same as expected by the MissionDirector
		#
		ports_and_callbacks.append((9983, private___rcv_msg))
		slistener.start(ports_and_callbacks, False)
	
	global slistener_msgs
	msg = slistener_msgs
	slistener_msgs = []
	return msg


#---------------------------------------------------------------------

def private___rcv_msg(msg):
	global slistener_msgs
	slistener_msgs.append(msg)


def private___dispatch_msg(msg,port):
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.connect(("localhost",port))
		s.send(msg)
		s.close()
	except:
		print "message failed to send: \""+str(msg)+"\""
