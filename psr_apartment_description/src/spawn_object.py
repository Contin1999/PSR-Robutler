#!/usr/bin/env python3

import random

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
import argparse

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('-object',required=False,type=str, help='name of the object to spawn, possible objects: book_pink beer parrot_bebop_2 hammer lamp_table_small mug_beer newspaper_3 can_fanta donut_1')
    parser.add_argument('-room', nargs= '+' ,required=False,type=str, help='room and location of the spawn, possible combinations: double_room(bed or besides_cabinet) single_bedroom(over_fridge_cabinet) study_room(over_cabinet_corner) bathroom(over_blue_bin_cuboid) livingroom(over_main_table)')

    args = vars(parser.parse_args())
    
    rospy.init_node('insert_object',log_level=rospy.INFO)

    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('psr_apartment_description') + '/models/'
    # package_path = '/home/igino/.gazebo/models/'

    #Add here mode poses
    placements = []
    placements.append({'pose':Pose(position=Point(x=-5.69, y=4.37, z=0.6), orientation=Quaternion(x=0,y=0,z=0,w=1)),
                'room':'double_room', 'place': 'bed'})
    placements.append({'pose':Pose(position=Point(x=-7.33, y=5.29, z=0.62), orientation=Quaternion(x=0,y=0,z=0,w=1)),
                'room':'double_room', 'place': 'bedside_cabinet'})
    placements.append({'pose':Pose(position=Point(x=-1.921, y=4, z=0.4), orientation=Quaternion(x=0,y=0,z=0,w=1)),
                'room':'single_bedroom', 'place': 'over_fridge_cabinet'})
    placements.append({'pose':Pose(position=Point(x=-0.84, y=5.3, z=1.02), orientation=Quaternion(x=0,y=0,z=0,w=1)),
                'room':'study_room', 'place': 'over_cabinet_corner'})
    placements.append({'pose':Pose(position=Point(x=0.388, y=1.744, z=0.963), orientation=Quaternion(x=0,y=0,z=0,w=1)),
                'room':'bathroom', 'place': 'over_blue_bin_cuboid'})
    placements.append({'pose':Pose(position=Point(x=-4.87, y=-3.813, z=1.02), orientation=Quaternion(x=0,y=0,z=0,w=1)),
                'room':'living_room', 'place': 'over_main_table'})


    model_names = ['sphere_v', 'book_pink', 'beer', 'parrot_bebop_2', 'hammer','lamp_table_small','mug_beer','newspaper_3','can_fanta','donut_1']

    # Add here several models. All should be added to the robutler_description package
    if args['object'] is None:
        model_name = random.choice(model_names)
    else:
        model_name = args['object']

    f = open( package_path + model_name + '/model.sdf' ,'r')
    sdff = f.read()

    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

    if args['room'] is None:
        model_placement = random.choice(placements)
    elif len(args['room']) == 1: # only room specified
        for placement in placements:
            if args['room'][0] == placement['room']:
                model_placement = placement

    elif len(args['room']) == 2:
        for placement in placements:
            if args['room'][0] == placement['room'] and args['room'][1] == placement['place']:
                model_placement = placement
    else :
        print('incorrect argument -room, random position will be used')
        model_placement  = random.choice(placements)

    name = model_name + '_in_' + model_placement['place'] + '_of_' + model_placement['room']
    print(name)
    spawn_model_prox(name, sdff, model_name, model_placement['pose'], "world")