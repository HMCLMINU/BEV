#!/usr/bin/env python3

import glob
import os
import sys
from visualization_msgs.msg import MarkerArray, Marker

try:
    sys.path.append(glob.glob('/home/hmcl/carla10/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import math
import random
import rospy

# from __init__ import *
from cv2 import cv2 as cv
from __init__ import (
    BirdViewProducer,
    BirdView,
    DEFAULT_HEIGHT,
    DEFAULT_WIDTH,
    BirdViewCropType,
)
from mask import PixelDimensions

STUCK_SPEED_THRESHOLD_IN_KMH = 3
MAX_STUCK_FRAMES = 30

fourcc = cv.VideoWriter_fourcc(*'MJPG')
image_save_path = '/home/hmcl/carla-birdeye-view/carla_birdeye_view/autodata'
n_seq = len(os.listdir(image_save_path))
track_cleanup = []

# class carla_ros():
#     def __init__(self) -> None:
#         rospy.init_node('BEV', anonymous=True)
#         rospy.Subscriber("/global_waypoints_rviz", MarkerArray, self.tarjCallback)
    
#     def tarjCallback(self, msg):
#         pose = msg.markers
#         # print("{}".format(pose))
#         rospy.spin()
        
def get_speed(actor: carla.Actor) -> float:
    """in km/h"""
    vector: carla.Vector3D = actor.get_velocity()
    MPS_TO_KMH = 3.6
    return MPS_TO_KMH * math.sqrt(vector.x ** 2 + vector.y ** 2 + vector.z ** 2)

def find_ego(world):
    while True:
        actorlist = world.get_actors().filter('vehicle.toyota.prius')                
        for actor in actorlist:
            ego_id = actor.id
            ego_candidate = world.get_actor(ego_id)
            role_name = ego_candidate.attributes['role_name']
            if role_name == 'ego_vehicle':
                print("ego id : {}".format(ego_id))
                real_ego = ego_id
        if ego_id == 0:
            pass
        else:
            print("got ego id")
            break

    return real_ego

def main():
    client = carla.Client("192.168.0.121", 2000)
    client.set_timeout(3.0)
    world = client.get_world()
    map = world.get_map()
    spawn_points = map.get_spawn_points()
    blueprints = world.get_blueprint_library()
    # settings = world.get_settings()
    # settings.synchronous_mode = True
    # settings.no_rendering_mode = True
    # settings.fixed_delta_seconds = 1 / 10.0
    # world.apply_settings(settings)

    agent = world.get_actor(find_ego(world))
    agent.set_autopilot(True)

    birdview_producer = BirdViewProducer(
        client,
        PixelDimensions(width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT),
        pixels_per_meter=4,
        crop_type=BirdViewCropType.FRONT_AND_REAR_AREA,
        render_lanes_on_junctions=False,
    )
    img_count = 1
    print('starting new seq')
    global n_seq
    n_seq += 1
    track_cleanup.append(n_seq)
    if not os.path.exists(os.path.join(image_save_path,str(n_seq))):
        os.makedirs(os.path.join(image_save_path,str(n_seq)))

    while True:
        # world.tick()
        birdview: BirdView = birdview_producer.produce(agent_vehicle=agent)
        bgr_img = cv.cvtColor(BirdViewProducer.as_rgb(birdview), cv.COLOR_BGR2RGB)
        # NOTE imshow requires BGR color model
        cv.imshow("BirdView RGB", bgr_img)

        bgr_img = bgr_img[:,:,:3][:]
        cv.imwrite(os.path.join(image_save_path, str(n_seq),'{}.png'.format(img_count)), bgr_img)
        if img_count % 15 == 0:
            n_seq += 1
            img_count = 0
            if not os.path.exists(os.path.join(image_save_path,str(n_seq))):
                os.makedirs(os.path.join(image_save_path,str(n_seq)))

        # cv.imwrite(os.path.join(image_save_path, '{}.png'.format(img_count)),bgr_img)
        img_count += 1

        # Play next frames without having to wait for the key
        key = cv.waitKey(10) & 0xFF
        if key == 27:  # ESC
            break
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
