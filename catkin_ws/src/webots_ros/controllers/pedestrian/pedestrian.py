from controller import Supervisor
import optparse

class Pedestrian (Supervisor):

    def __init__(self):
        self.time_step=1000
        self.path="../../src/change_pkg/positions"
        Supervisor.__init__(self)

    def run(self):
        opt_parser = optparse.OptionParser()
        opt_parser.add_option("--step", type=int, help="Specify time step (otherwise world time step is used)")
        options, args = opt_parser.parse_args()
        if options.step and options.step > 0:
            self.time_step = options.step
        self.root_node_ref = self.getSelf()
        self.root_translation_field = self.root_node_ref.getField("translation")
        self.name = self.root_node_ref.getField("name").getSFString()
        print(self.name)
        
        while not self.step(self.time_step) == -1:
            position=self.root_translation_field.getSFVec3f()
            with open('{}/{}.txt'.format(self.path,self.name), 'w') as f:
                f.write("{},{}".format(position[0],position[2]))
            



controller = Pedestrian()
controller.run()
