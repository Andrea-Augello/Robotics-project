#!/usr/bin/env python
from controller import Robot, Camera
import math 

def main():
    TIME_STEP = 16
    robot = Robot()
    camera = robot.getCamera("camera")
    camera.enable(TIME_STEP)
    camera.recognitionEnable(TIME_STEP)
    while robot.step(TIME_STEP) != -1:
        objects=camera.getRecognitionObjects() 
        objects=[RecognitionObject(i.get_id(),i.get_position(),i.get_orientation(),i.get_size(),i.get_position_on_image(),i.get_size_on_image(),i.get_number_of_colors(),i.get_colors(),str(i.get_model())) for i in objects]
        positions=[get_seed_from_object(o) for o in objects if "pedestrian" in o.model] #if o.model=="pedestrian"
        polar_positions=[abs_cartesian_to_polar(p) for p in positions]
        write(polar_positions)
        
def get_seed_from_object(recognition_object):
    return recognition_object.position

def write(positions):
    path="../../src/change_pkg/observations"
    with open('{}/observations.txt'.format(path), 'w') as f:
        for seed in positions:
            f.write("{}\n".format(seed))

def abs_cartesian_to_polar(p):
    """Accepts a point given in cartesian coordinates relative to the world
    frame of reference and converts it to polar coordinates in the robot
    frame of reference

    :p:   Cartesian coordinates in the (x, y) format
    :returns:  Cartesian coordinates in the (Rho, Theta) format

    """
    x = p[0]
    y = -p[2]
    angle = -(math.atan2(x,y)*180)/math.pi
    angle = angle if angle < 180 else angle -360
    return (math.hypot(x,y), angle)            
                    

class RecognitionObject():
    def __init__(self,id,position,orientation,size,position_on_image,size_on_image,number_of_colors,colors,model):
        self.id=id
        self.position=position
        self.orientation=orientation
        self.size=size
        self.position_on_image=position_on_image
        self.size_on_image=size_on_image
        self.number_of_colors=number_of_colors
        self.colors=colors
        self.model=model

    def __str__(self):
        return 'id:{}\nposition:{}\norientation:{}\nsize:{}\nposition_on_image:{}\nsize_on_image:{}\nnumber_of_colors:{}\ncolors:{}\nmodel:{}\n'.format(self.id,self.position,self.orientation,self.size,self.position_on_image,self.size_on_image,self.number_of_colors,self.colors,self.model)     

if __name__ == '__main__':
    main()