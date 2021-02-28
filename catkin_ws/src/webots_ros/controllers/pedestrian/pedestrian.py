from controller import Supervisor
import optparse
import random

class Pedestrian (Supervisor):

    def __init__(self):
        self.time_step=1000
        self.path="../../src/change_pkg/positions"
        Supervisor.__init__(self)

    def run(self):
        x_room=10
        y_room=10
        height=1.27
        x=random.random()*(x_room-1)+0.5-x_room/2
        y=random.random()*(y_room-1)+0.5-y_room/2
        while self.in_trajectory(x,y):
            x=random.random()*(x_room-1)+0.5-x_room/2
            y=random.random()*(y_room-1)+0.5-y_room/2
        opt_parser = optparse.OptionParser()
        opt_parser.add_option("--step", type=int, help="Specify time step (otherwise world time step is used)")
        options, args = opt_parser.parse_args()
        if options.step and options.step > 0:
            self.time_step = options.step
        self.root_node_ref = self.getSelf()
        self.root_translation_field = self.root_node_ref.getField("translation")
        self.name = self.root_node_ref.getField("name").getSFString()
        self.root_translation_field.setSFVec3f([y,height,x])
        
        while not self.step(self.time_step) == -1:
            position=self.root_translation_field.getSFVec3f()
            with open('{}/{}.txt'.format(self.path,self.name), 'w') as f:
                f.write("{},{}".format(-position[2],position[0]))
            

    def in_trajectory(self,x,y):
        r=2.5
        t=1.2
        return not self.in_square(r-t,x,y) and self.in_square(r+t,x,y)

    def in_square(self,p,x,y):
        return -p<x<p and -p<y<p    

controller = Pedestrian()
controller.run()
