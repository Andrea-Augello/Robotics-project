#!/usr/bin/env python3
from controller import Robot, Camera

def main():
    TIME_STEP = 32
    robot = Robot()
    camera = robot.getCamera("camera")
    camera.enable(TIME_STEP)
    camera.recognitionEnable(TIME_STEP)
    positions=[]
    while robot.step(TIME_STEP) != -1:
        objects=camera.getRecognitionObjects()
        objects=[RecognitionObject(i.get_id(),i.get_position(),i.get_orientation(),i.get_size(),i.get_position_on_image(),i.get_size_on_image(),i.get_number_of_colors(),i.get_colors(),i.get_model()) for i in objects]
        for o in objects:
            positions.append(get_seed_from_object(o))
        write(positions)
        
def get_seed_from_object(recognition_object):
    return recognition_object.position

def write(positions):
    path="../../src/change_pkg/SEEDS"
    with open('{}/seeds.txt'.format(path), 'w') as f:
        for seed in positions:
            f.write("%s\n" % seed)
                                  

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