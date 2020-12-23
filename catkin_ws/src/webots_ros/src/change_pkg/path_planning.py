import rospy
from change_pkg.vision import *
import change_pkg.clustering as clst

class Path_planner:

    """Docstring for Path_planner. """

    def __init__(self, robot, odometry):
        """Accepts a robot and an odometry object

        :robot: The robot this module belongs to
        :odometry: The odometry object used to estimate the position

        """
        self._robot = robot
        self._odometry = odometry
        self.people_coords=[]

    def update_people_coords(self, new_cords):
        """
        :new_cords: The updated coordinates
        :returns: TODO

        """
        theta = self._odometry.theta
        x = self._odometry.x
        y = self._odometry.y
        cartesian_coords = []
        for p in new_cords:
            angle=p[1]+theta
            cartesain = []
            cartesian.append(p[0]*math.sin(math.pi*angle/180))
            cartesian.append(p[0]*math.cos(math.pi*angle/180))
            cartesian_coords.append(cartesain)
        # TODO some more sophisticated analysis here, e.g. iterative closest
        # point estimate for data points that can be linked with past data,
        # keeping the old position as best guess for points that have been
        # lost. Possibly update position estimate with triangulation
        self.people_coords = cartesain

    def find_clusters(self):
        """Based on the self.people_coords finds clusters and returns them.
        This may not be the best place for this function, may move to vision
        module or a specific "reasoning" object.

        :returns: Clusters, either as the points in each cluster as a list of
            lists, or a centroid.

        """
        # TODO find reasonable parameters for the Density Based Scan clustering
        # algorithm
        pass
