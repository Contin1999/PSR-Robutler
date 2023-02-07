#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import rospy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

from functools import partial
import subprocess

from std_msgs.msg import String

server = None
marker_pos = 0

menu_handler = MenuHandler()

pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

rooms = ['kitchen','living_room','small_bathroom','bathroom','double_room','single_room','study_room','hallway'] # from go_to_specific_point_on_map.py

locations = ["living_room", "double_room", "single_room", "study_room", "bathroom"] # from spawn_object.py

objects = ['sphere_v', 'book_pink', 'beer', 'parrot_bebop_2', 'hammer','lamp_table_small','mug_beer','newspaper_3','can_fanta','donut_1','person_standing']# from spawn_object.py

missions = [
            "find a purple sphere",
            "check if the pc is in the office",
            "check if there is someone in the living room"
            ]

#-------------------------------------------------------
# definition of callback--------------------------------
#-------------------------------------------------------

def marker(text):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = marker.TEXT_VIEW_FACING
    marker.scale.z = 0.35
    marker.color.a = 1.0
    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 1
    marker.pose.orientation.w = 1.0
    marker.text = text
    marker.lifetime = rospy.Duration(2)
    
    return marker

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 0.0 # alpha to zero-->not visible

    return marker

def makeEmptyMarker( dummyBox=True ):
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    # int_marker.pose.position.y = -3.0 * marker_pos
    # marker_pos += 1
    int_marker.scale = 1
    return int_marker

def makeMenuMarker( name ):
    int_marker = makeEmptyMarker()
    int_marker.name = name

    control = InteractiveMarkerControl()

    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True

    control.markers.append( makeBox( int_marker ) )
    int_marker.controls.append(control)

    server.insert( int_marker )

def mode_navigate(room, feedback):
    text = "Go to "+str(room)
    rospy.loginfo(text)
    text_show = marker(text)
    pub.publish(text_show)

    bashCommand = "rosrun robutler_navigation go_to_specific_point_on_map.py -room "+str(room)
    nav_process = subprocess.Popen(bashCommand.split())

def mode_detect(type, feedback):
    global h_yolo, h_color
    global  proc_yolo, proc_color
    state_yolo = menu_handler.getCheckState( h_yolo )
    state_color = menu_handler.getCheckState( h_color )

    if type == 'yolo':
        if state_yolo == MenuHandler.CHECKED:
            text = "detection "+str(type)+' deactivated'
            rospy.loginfo(text)
            text_show = marker(text)
            pub.publish(text_show)

            menu_handler.setCheckState( h_yolo, MenuHandler.UNCHECKED )
            proc_yolo.kill()
        else:
            text = "detection "+str(type)+' activated'
            rospy.loginfo(text)
            text_show = marker(text)
            pub.publish(text_show)

            menu_handler.setCheckState( h_yolo, MenuHandler.CHECKED )

            bashCommand = "roslaunch robutler_detection yolov7.launch"
            proc_yolo = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)

    if type == 'by_color':
        if state_color == MenuHandler.CHECKED:
            text = "detection "+str(type)+' deactivated'
            rospy.loginfo(text)
            text_show = marker(text)
            pub.publish(text_show)

            menu_handler.setCheckState( h_color, MenuHandler.UNCHECKED )
            proc_color.kill()
        else:
            text = "detection "+str(type)+' activated'
            rospy.loginfo(text)
            text_show = marker(text)
            pub.publish(text_show)

            menu_handler.setCheckState( h_color, MenuHandler.CHECKED )

            bashCommand = "rosrun robutler_detection detect_color.py"
            proc_color  = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)

    menu_handler.reApply( server )
    server.applyChanges()

def mode_photo(feedback):
    text = 'take a photo...'
    rospy.loginfo(text)
    text_show = marker(text)
    pub.publish(text_show)

    bashCommand = "rosrun robutler_detection take_photo.py"
    photo_process = subprocess.Popen(bashCommand.split())#, stdout=subprocess.PIPE)
    # output, error = process.communicate()
    # print(output)

def mode_spawn(feedback):
    global h_object_last, h_location_last

    object = menu_handler.getTitle(h_object_last)
    location = menu_handler.getTitle(h_location_last)

    text = 'spawn '+str(object) + ' in ' + str(location)
    rospy.loginfo(text)
    text_show = marker(text)
    pub.publish(text_show)

    bashCommand = "rosrun psr_apartment_description spawn_object.py -object "+str(object) + " -room "+str(location)
    spawn_process = subprocess.Popen(bashCommand.split())

def mode_mission(mission,feedback):
    text = 'elaborate mission: '+str(mission)
    rospy.loginfo(text)
    text_show = marker(text)
    pub.publish(text_show)

    pub_mission.publish(mission)

def change_check_object(object, feedback):
    global h_object_last
    print(h_object_last)
    menu_handler.setCheckState( h_object_last, MenuHandler.UNCHECKED )
    h_object_last = feedback.menu_entry_id
    menu_handler.setCheckState( h_object_last, MenuHandler.CHECKED )

    rospy.loginfo("Switching to object: " + str(object))
    menu_handler.reApply( server )
    server.applyChanges()

def change_check_location(location, feedback):
    global h_location_last

    menu_handler.setCheckState( h_location_last, MenuHandler.UNCHECKED )
    h_location_last = feedback.menu_entry_id
    menu_handler.setCheckState( h_location_last, MenuHandler.CHECKED )

    rospy.loginfo("Switching to location: " + str(location))
    menu_handler.reApply( server )
    server.applyChanges()

def callback_sub_mission(data):
    text = data.data
    text_show = marker(text)
    pub.publish(text_show)

def initMenu():

    # NAVIGATION
    global h_room_last
    nav_handler = menu_handler.insert( "navigation" )
    for room in rooms:
        s = "go to: " + str(room)
        h_room_last = menu_handler.insert( s, parent=nav_handler, callback=partial(mode_navigate,room) )

    # DETECTION
    global h_yolo, h_color
    detect_handler = menu_handler.insert( "detection" )
    h_yolo = menu_handler.insert( 'yolo', parent=detect_handler, callback=partial(mode_detect,'yolo') )
    menu_handler.setCheckState( h_yolo, MenuHandler.UNCHECKED)
    h_color = menu_handler.insert( 'detection by color', parent=detect_handler, callback=partial(mode_detect,'by_color') )
    menu_handler.setCheckState( h_color, MenuHandler.UNCHECKED)

    photo_handler = menu_handler.insert( "take a photo" , callback = mode_photo)

    # SPAWN OBJECTS
    spawn_handler = menu_handler.insert( "spawn object" )

    global h_object_last
    spawn_obj_handler = menu_handler.insert( 'object', parent=spawn_handler)
    for object in objects:
        h_object_last = menu_handler.insert( str(object), parent = spawn_obj_handler, callback=partial(change_check_object, object))
        menu_handler.setCheckState( h_object_last, MenuHandler.UNCHECKED)
    # check the very last entry
    menu_handler.setCheckState( h_object_last, MenuHandler.CHECKED )

    global h_location_last
    spawn_loc_handler = menu_handler.insert( 'location', parent=spawn_handler)
    for location in locations:
        h_location_last = menu_handler.insert( str(location), parent = spawn_loc_handler, callback=partial(change_check_location, location))
        menu_handler.setCheckState( h_location_last, MenuHandler.UNCHECKED)
    # check the very last entry
    menu_handler.setCheckState( h_location_last, MenuHandler.CHECKED )

    spawn_spawn_handler = menu_handler.insert('spawn!',parent = spawn_handler, callback=mode_spawn)

    # MISSIONS 
    mission_handler = menu_handler.insert( "missions" )
    for mission in missions:
        entry = menu_handler.insert( str(mission), parent = mission_handler, callback=partial(mode_mission,mission) )


if __name__=="__main__":
    rospy.init_node("menu")
    
    server = InteractiveMarkerServer("menu")

    initMenu()

    # mission using the ros topic
    pub_mission = rospy.Publisher('mission', String, queue_size=10)
    sub_mission = rospy.Subscriber('mission_state',String,queue_size = 10, callback= callback_sub_mission)
    
    makeMenuMarker( "marker_robot" )

    menu_handler.apply( server, "marker_robot" )
    server.applyChanges()

    rospy.spin()
