import rospy
import rosservice
import pprint
import json
from geofrenzy.srv import entitlement_service
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import NavSatFix

dwell_dict = {}
green_dict = {}
red_dict = {}
blue_dict = {}
alpha_dict = {}
fix = False;
interval=1.0
msg_list=[]


def gps_callback(data):
	global fix
	if (data.status.status <= 0):
		fix=True;
	else:
		fix=False
	pprint.pprint(fix)

def callback(data):
	global msg_list
	dwell_total = False;
	red_total = 0
	green_total = 0
	blue_total = 0
	alpha_total = 0
	count_total = 0
	json_data = json.loads(data.data)
	pprint.pprint(json_data)
	dwell_dict[json_data["class_idx"]] = json_data["dwell"]
	red_dict[json_data["class_idx"]] = json_data["color_red"]
	green_dict[json_data["class_idx"]] = json_data["color_green"]
	blue_dict[json_data["class_idx"]] = json_data["color_blue"]
	alpha_dict[json_data["class_idx"]] = json_data["color_alpha"]

##each time through the callback we build a a full msg_list with the appropriate topics to publish in the main loop 
	#for dwell in dwell_dict:
		#dwell_total = dwell_total or dwell_dict[dwell]
	##	print "*****" + dwell + "****"
		#if dwell_dict[dwell]:
				#count_total = count_total + 1
				#msg = ColorRGBA(float(json_data["color_red"]),float(json_data["color_green"]),float(json_data["color_blue"]),(float(json_data["color_alpha"])/255))
				#msg_list.append(msg)
				#print "====================="
			
##	if json_data["dwell"]:
##		msg = ColorRGBA(float(json_data["color_red"]),float(json_data["color_green"]),float(json_data["color_blue"]),(float(json_data["color_alpha"])/255))
##		pub.publish(msg)
## if the GPS has a fix but no fences, we put the light on full brightness
	#if (len(msg_list)<1):
		#print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>no fence<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,"
		#msg_list=[]
		#msg = ColorRGBA(255,255,255,1)
		#msg_list.append(msg)
		#pub.publish(msg)
		#interval=1
## if the GPS has no fix then we just barely put the light on just to show we are alive
	#if not fix:
		#print "No Fix"
		#msg_list=[]
		#msg = ColorRGBA(10,10,10,1)
		#msg_list.append(msg)
		#pub.publish(msg)
		#interval=1
	#pprint.pprint(dwell_total);
	#red_avg = red_total/count_total
	#green_avg = green_total/count_total
	#blue_avg = blue_total/count_total
	#alpha_avg = alpha_total/count_total
	#print count_total,dwell_total,red_avg,green_avg,blue_avg,alpha_avg
	#msg = ColorRGBA(float(red_avg),float(green_avg),float(blue_avg),(float(alpha_avg)/255))
	#pub.publish(msg)
	


rospy.init_node("gf_color_mux");
services = rosservice.get_service_list()
services = rosservice.rosservice_find("geofrenzy/entitlement_service")
topics = rospy.get_published_topics('/geofrenzy')
service_list = []
for service in services:
	entitlement_srv = rospy.ServiceProxy(service,entitlement_service)
	response = entitlement_srv().ent_base
	if response == "color":
		service = service+"/dwell/json"
		service_list.append(service) 
		rospy.Subscriber(service, String, callback)
gps_service_name="fix"
rospy.Subscriber(gps_service_name,NavSatFix,gps_callback)
pprint.pprint(service_list)
pub = rospy.Publisher('Blink',ColorRGBA,queue_size=2)

while not rospy.is_shutdown():
	msg_list=[]
	#each time through the callback we build a a full msg_list with the appropriate topics to publish in the main loop 
	for dwell in dwell_dict:
		#dwell_total = dwell_total or dwell_dict[dwell]
	#	print "*****" + dwell + "****"
		if dwell_dict[dwell]:
				#count_total = count_total + 1
				#msg = ColorRGBA(float(json_data["color_red"]),float(json_data["color_green"]),float(json_data["color_blue"]),(float(json_data["color_alpha"])/255))
				msg = ColorRGBA(
					float(red_dict[dwell]),
					float(green_dict[dwell]),
					float(blue_dict[dwell]),
					(float(alpha_dict[dwell])/255))
				msg_list.append(msg)
				print "====================="
			
#	if json_data["dwell"]:
#		msg = ColorRGBA(float(json_data["color_red"]),float(json_data["color_green"]),float(json_data["color_blue"]),(float(json_data["color_alpha"])/255))
#		pub.publish(msg)
# if the GPS has a fix but no fences, we put the light on full brightness
	if (len(msg_list)<1):
		print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>no fence<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,"
		msg_list=[]
		msg = ColorRGBA(255,255,255,1)
		msg_list.append(msg)
		pub.publish(msg)
		interval=1
# if the GPS has no fix then we just barely put the light on just to show we are alive
	if not fix:
		print "No Fix"
		msg_list=[]
		msg = ColorRGBA(10,10,10,1)
		msg_list.append(msg)
		pub.publish(msg)
		interval=1
	mycount=len(msg_list)
	if mycount != 0:
		r=rospy.Rate(interval)
	else:
		r=rospy.Rate(1)
	pprint.pprint(msg_list)
	for each in msg_list:
		pub.publish(each)
		print "interval="+str(interval)
		r.sleep()
		
#rospy.spin()

