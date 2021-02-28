from controller import Supervisor

class Referee (Supervisor):

    def __init__(self):
        self.time_step=1000
        self.path="../../src/change_pkg/robot_position"
        Supervisor.__init__(self)

    def run(self):
        while not self.step(self.time_step) == -1:
            with open('{}/robot_position.txt'.format(self.path), 'r') as f:
                [flag,x,y]=f.readline().split(",")
            if int(flag):    
                self.root_node_ref = self.getFromDef("TiagoIron")
                self.root_translation_field = self.root_node_ref.getField("translation")
                self.root_translation_field.setSFVec3f([float(x),0.095,float(y)])
                f.close()
                with open('{}/robot_position.txt'.format(self.path), 'w') as f:
                    f.write("0,{},{}\n".format(x,y))
                    f.close()

controller = Referee()
controller.run()
