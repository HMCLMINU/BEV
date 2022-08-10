import os,sys,random,time
import glob

try:
    sys.path.append(glob.glob('/home/hmcl/carla10/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import numpy as np
import cv2

#settings
IM_W,IM_H = (420,280)
image_save_path ='autodata'
seq_len = 15

#create main carla objects
client = carla.Client('192.168.0.121',2000)
client.set_timeout(5)
world = client.get_world()

blueprint_library = world.get_blueprint_library()

class Carla_session:

    def __init__(self):
        self.actors = []
        self.counter = 0
        self.n_seq = len(os.listdir(image_save_path))
        self.collision_flag = False
        self.episode_images = []
        self.track_cleanup =[]
        self.env_actors = []

    def add_actors(self):

        start_point = random.choice(world.get_map().get_spawn_points())

        #set vehicle
        vehicle_bp = blueprint_library.find('vehicle.ford.mustang')
        self.vehicle = world.spawn_actor(vehicle_bp,start_point)
        #self.vehicle.set_autopilot(True)

        #get and set sensors
        collision_sensor_bp = blueprint_library.find('sensor.other.collision')
        lane_invasion_sensor_bp = blueprint_library.find('sensor.other.lane_invasion')

        sensor_location = carla.Transform(carla.Location(x=4,y=0,z=2.5))
        self.camera = world.spawn_actor(camera_sensor_bp, sensor_location, attach_to = self.vehicle)
        self.collision_sensor = world.spawn_actor(collision_sensor_bp, sensor_location, attach_to = self.vehicle)
        #self.lane_invasion_sensor = world.spawn_actor(lane_invasion_sensor_bp, sensor_location, attach_to = self.vehicle)
        self.actors.extend([self.vehicle,self.camera])
        self.camera.listen(lambda image: self.add_image(image))
        #self.collision_sensor.listen(lambda collision: self.end_seq(collision,'collision'))  

        #self.lane_invasion_sensor.listen(lambda lane_inv: self.end_seq(lane_inv,'crossed lane'))

    def start_new_seq(self):
        self.add_actors()
        self.collision_flag = False
        print('starting new seq')
        self.counter = 0
        self.n_seq+=1
        self.track_cleanup.append(self.n_seq)
        if not os.path.exists(os.path.join(image_save_path,str(self.n_seq))):
            os.makedirs(os.path.join(image_save_path,str(self.n_seq)))

    def add_image(self, image):
        self.counter += 1
        img = np.reshape(image.raw_data,(IM_H,IM_W,4))
        img = img[:,:,:3][:]
        #self.episode_images.append(img)
        
        cv2.imwrite(os.path.join(image_save_path, str(self.n_seq),'{}.png'.format(self.counter)),img)
        if self.counter%15 == 0:
            self.n_seq += 1
            self.counter = 0
            if not os.path.exists(os.path.join(image_save_path,str(self.n_seq))):
                os.makedirs(os.path.join(image_save_path,str(self.n_seq)))

        #cv2.imshow("live",img)
        #cv2.waitKey(1)

    def delete_images(self):
        imagestodelete = self.counter-seq_len
        for i in range(imagestodelete):
            os.remove(os.path.join(image_save_path,str(self.n_seq),'{}.png'.format(i+1)))

    def save_images(self):
        #print(os.path.join(image_save_path,str(self.n_seq),'{}.png'.format(self.counter)))
        for ind,img in enumerate(self.episode_images[-seq_len:]):
            cv2.imwrite(os.path.join(image_save_path,str(self.n_seq),'{}.png'.format(ind)),img)
    
    def end_seq(self,cause_obj,cause):
        self.destroy_actors()
        self.collision_flag =True
        print("collision happened")
        self.delete_images()

    def destroy_actors(self):
        for actor in self.actors:
            actor.destroy()

        #self.save_images()
        self.actors = []
        #self.episode_images =[]
    
    def get_directions(self):
        thr = random.choice([0.8,0.7,0.6])
        steer = random.choice([-0.3,0.0,0.0,0.0,0.3,0.1,-0.1])
        return carla.VehicleControl(thr,steer)  
       
    def drive_around(self,episodes):
        try:
            self.start_new_seq()
        except:
            pass

        self.vehicle.set_autopilot(True)
        time.sleep(60*60*2)