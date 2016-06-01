from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import requests
import json
import datetime
import os

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')

parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
"""
parser.add_argument('--points','--list', nargs='*', help='<Required> Set flag', required=True,type=float)
"""
parser.add_argument('--points','--list', nargs='*', help='<Required> Set flag', required=True,type=float)

parser.add_argument('--id',help="id du drone, on va l'utiliser pour appeller put de l'API Deones")

parser.add_argument('--intervention',help="id de l'intervention")

parser.add_argument('--instance',help="instance de  dronekit")


args = parser.parse_args()
val=int(args.instance)
port=5760+10*val
if not args.connect:
        connection_string='tcp:127.0.0.1:'+(str(port))
	#connection_string = '127.0.0.1:14550'


destination=args.points
print "destination", destination
print "la taille de destination", len(destination)
lesPoints=[]
taille=len(destination)

def chargerDestination(des):
	res=des
	if ((des[0]==des[taille-3])and (des[1]==des[taille-2]) and (des[2]==des[taille-1])):
		res=des[0:(taille-3)]
	if len(res)!=0:
		i=0
		while(i!=len(res)):	
		    lesPoints.append(LocationGlobalRelative(res[i],res[i+1],res[i+2]))
		    i=i+3
	return res

vehicle = connect(connection_string, wait_ready=True)
vehicle.airspeed = 60


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint



def distance_to_current_waypoint2(i):
    lat = vehicle.commands[i].x
    lon = vehicle.commands[i].y
    alt = vehicle.commands[i].z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)	    
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.



def adds_mission(lesPoints):
	
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """	

    cmds = vehicle.commands
    cmds.download()
    print " Clear any existing commands"
    cmds.clear() 

    
    print " Define/add new commands."
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    #cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    for i in range(0,len(lesPoints)):  
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lesPoints[i].lat, lesPoints[i].lon, lesPoints[i].alt))
	print " Upload new commands to vehicle"
    #cmds.upload()
   
    print len(cmds)
    cmds.upload()



def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

        
    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print "Reached target altitude"
            break
        time.sleep(1)

#url de l 'API
idDrone=args.id
idIntervention=args.intervention
url="http://devprojetm2gla.istic.univ-rennes1.fr:12349/api/Drones/"+idDrone


url2="http://devprojetm2gla.istic.univ-rennes1.fr:12353/api/Images/upload?"
headers = {"content-type": "application/json"}

def genererkml(fichierkml,longitude,latitude):
	kml = (
   '<?xml version="1.0" encoding="UTF-8"?>\n'
   '<kml xmlns="http://www.opengis.net/kml/2.2">\n'
   '<Document>\n'
   '<name>Test camera KML</name>\n'
   '<Camera>\n'
   '<longitude>%s</longitude>\n'
   '<latitude>%s</latitude>\n'
   '<altitude>200.0</altitude>\n'
   '<heading>280.873558304</heading>\n'
   '<tilt>0</tilt>\n'
   '<altitudeMode>absolute</altitudeMode>\n'
   '</Camera>\n'
   '</Document>\n'
   '</kml>'
   ) %(longitude, latitude)
	mon_fichier = open("fichier.kml", "w")
	mon_fichier.write(kml)
	mon_fichier.close()
	
#programme principale        
print 'Create a new mission (for current location)'
destination=chargerDestination(destination)
adds_mission(lesPoints)

arm_and_takeoff(10)

vehicle.commands.next=0

vehicle.mode = VehicleMode("AUTO")


def missionCercle(link):
	lien=link
	i=0;
	while True:
	    print 'missionCercle'
	    nextwaypoint=vehicle.commands.next
	    #print 'Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint())
	    print 'Distance to waypoint (%s): %s' % (i+1, distance_to_current_waypoint2(i)) 
	    payload = {'location':{'geopoint':{'latitude':vehicle.location.global_frame.lat,'longitude':vehicle.location.global_frame.lon,'altitude':vehicle.location.global_frame.alt}}}
	    r=requests.put(url, data=json.dumps(payload),headers=headers)
	    print "latitude", vehicle.location.global_frame.lat
	    print "longitude", vehicle.location.global_frame.lon
	    print "altitude", vehicle.location.global_frame.alt
	    print "code status :", r.status_code
	    genererkml("fichier.kml",vehicle.location.global_frame.lon,vehicle.location.global_frame.lat)
	    
	    if distance_to_current_waypoint2(i)<250:
		print 'Distance to waypoint (%s): %s' % (i, distance_to_current_waypoint2(i)) 
		#t= datetime.datetime.now().strftime("%Y-%m-%d-%H-%M:%S")
		t=time.strftime("%A %d %B %Y %H:%M:%S")
	        latt=str(vehicle.location.global_frame.lat)
		longg=str(vehicle.location.global_frame.lon)
		print 'vehicle.location.global_frame.lat :',vehicle.location.global_frame.lat
		print 'vehicle.location.global_frame.lon :',vehicle.location.global_frame.lon	
	    	link=lien+"intervention="+idIntervention+"&drone="+idDrone+"&latitude="+latt+"&longitude="+longg+"&takenAt="+t
		print link
		os.system('scrot image.jpg')
		files = {'media':  open('image.jpg', 'r')}
		#ch='curl -F image.jpeg '+link
		#os.system('ch')
		r = requests.post(link, files=files)
		
		print "code status pour l'envoie de photo:", r.status_code
		i=(i+1)%len(lesPoints)

		
	    if nextwaypoint==len(lesPoints) and distance_to_current_waypoint()<2: 
		adds_mission(lesPoints)
		arm_and_takeoff(10)
		vehicle.commands.next=1
		vehicle.mode = VehicleMode("AUTO")	
		vehicle.commands.next=1
	    time.sleep(0.5)	

def missionAllerRetour(link):
	lien=link
	i=0;
	tab=list (lesPoints)
	while True:
	    print 'missionAllerRetour'
	    nextwaypoint=vehicle.commands.next
	    #print 'Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint())
	    print 'Distance to waypoint (%s): %s' % (i+1, distance_to_current_waypoint2(i))
            payload = {'location':{'geopoint':{'latitude':vehicle.location.global_frame.lat,'longitude':vehicle.location.global_frame.lon,'altitude':vehicle.location.global_frame.alt}}}
	    r=requests.put(url, data=json.dumps(payload),headers=headers)
	    print "latitude", vehicle.location.global_frame.lat
	    print "longitude", vehicle.location.global_frame.lon
	    print "altitude", vehicle.location.global_frame.alt
	    print "code status :", r.status_code
	    genererkml("fichier.kml",vehicle.location.global_frame.lon,vehicle.location.global_frame.lat)


	    if distance_to_current_waypoint2(i)<250:
		print 'Distance to waypoint (%s): %s' % (i, distance_to_current_waypoint2(i)) 
		#t= datetime.datetime.now().strftime("%Y-%m-%d-%H-%M:%S")
		t=time.strftime("%A %d %B %Y %H:%M:%S")
	        latt=str(vehicle.location.global_frame.lat)
		longg=str(vehicle.location.global_frame.lon)
		print 'vehicle.location.global_frame.lat :',vehicle.location.global_frame.lat
		print 'vehicle.location.global_frame.lon :',vehicle.location.global_frame.lon	
	    	link=lien+"intervention="+idIntervention+"&drone="+idDrone+"&latitude="+latt+"&longitude="+longg+"&takenAt="+t
		print link
		os.system('scrot image.jpg')
		files = {'media':  open('image.jpg', 'r')}
		#ch='curl -F image.jpeg '+link
		#os.system('ch')
		r = requests.post(link, files=files)
		
		print "code status pour l'envoie de photo:", r.status_code
		i=(i+1)%len(lesPoints)
	    if nextwaypoint==len(lesPoints) and distance_to_current_waypoint()<2:
		tab=list(reversed(tab))
		adds_mission(tab)
		arm_and_takeoff(10)
		vehicle.commands.next=1
		vehicle.mode = VehicleMode("AUTO")	
		vehicle.commands.next=1
	    time.sleep(1)	


if (taille==len(destination)):
	missionAllerRetour(url2)
else:	
	missionCercle(url2)	
	
    

#print 'Return to launch'
#vehicle.mode = VehicleMode("RTL")


#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()
"""
# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()
"""
