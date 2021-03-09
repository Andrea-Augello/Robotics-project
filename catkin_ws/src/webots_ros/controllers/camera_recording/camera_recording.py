#!/usr/bin/env python
from controller import Robot, Camera

def main():
    TIME_STEP = 32
    robot = Robot()
    camera = robot.getCamera("camera")
    camera.enable(TIME_STEP)
    camera.recognitionEnable(TIME_STEP)
    

    while robot.step(TIME_STEP) != -1:
        objects=camera.getRecognitionObjects()
        objects=[RecognitionObject(i.get_id(),i.get_position(),i.get_orientation(),i.get_size(),i.get_position_on_image(),i.get_size_on_image(),i.get_number_of_colors(),i.get_colors(),i.get_model()) for i in objects]
        rois=[get_roi_from_object(o) for o in objects]
        seeds=rois_to_seeds(rois)
        write(seeds)
        
def get_roi_from_object(recognition_object):
    #TODO to do
    return recognition_object.position  

def write(seeds):
    path="../../src/change_pkg/ROI"
    with open('{}/ROI.txt'.format(path), 'w') as f:
        for seed in seeds:
            f.write(seed)
                  

def rois_to_seeds(rois):
    #TODO to do
    pass                  

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