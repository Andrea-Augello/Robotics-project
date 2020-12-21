import rospy
import cv2
import math

robot = None

class Vision:
    def __init__(self, r):
        global robot
        robot=r    
        self.FOCAL_LENGTH   = 0.0036
        self.IMAGE_HEIGHT   = 480
        self.IMAGE_WIDTH    = 640
        self.SENSOR_HEIGHT  = 0.002549874    # Rough guess, to be refined
        self.SENSOR_HEIGHT  = 0.003399832    # Rough guess, to be refined
        self.VERTICAL_FOV   = 44.56046121921962
        self.HORIZONTAL_FOV = 57.29578
        self.current_frames = []
        self.current_rois = []
        self.show_roi = False
        self.show_image = True

    def obj_dist_w(self, p1, p2, real_width):
        diff = (p1[0]-p2[0],p1[1]-p2[1])
        pixel_height = math.sqrt(diff[1]**2 + diff[1]**2)
        return (real_width*self.FOCAL_LENGTH*self.IMAGE_WIDTH)\
                /(pixel_height*self.SENSOR_WIDTH)

    def obj_dist_h(self, p1, p2, real_height):
        diff = (p1[0]-p2[0],p1[1]-p2[1])
        pixel_height = math.sqrt(diff[0]**2 + diff[1]**2)
        return (real_height*self.FOCAL_LENGTH*self.IMAGE_HEIGHT)\
                /(pixel_height*self.SENSOR_HEIGHT)


    def obj_height(self, p1, p2, real_distance):
        diff = (p1[0]-p2[0],p1[1]-p2[1])
        pixel_height = abs(diff[1])
        return (real_distance*pixel_height*self.SENSOR_HEIGHT)\
                /(self.FOCAL_LENGTH*self.IMAGE_HEIGHT)


    def obj_width(self, p1, p2, real_distance):
        diff = (p1[0]-p2[0],p1[1]-p2[1])
        pixel_height = abs(diff[0])
        return (real_distance*pixel_height*self.SENSOR_WIDTH)\
                /(self.FOCAL_LENGTH*self.IMAGE_WIDTH)


    def point_polar_coords(self, p, rho):
        # ISO 80000-2:2019 convention: radial distance r, polar angle θ
        # (theta), and azimuthal angle φ (phi). The symbol ρ (rho) is often
        # used instead of r. Phi here is the elevation angle (i.e. measured
        # from the reference plane), not the inclination angle (measured from
        # the Z axis).
        theta = (p[0]/self.IMAGE_WIDTH-0.5)*self.HORIZONTAL_FOV
        phi = -(p[1]/self.IMAGE_HEIGHT-0.5)*self.VERTICAL_FOV
        return (rho, theta, phi)


    def point_coords(self, p, rho):
        img_center = (self.IMAGE_WIDTH/2,self.IMAGE_HEIGHT/2)
        x = obj_width( p, img_center,rho)*(-1 if p[0]<img_center[0] else 1)
        z = obj_height(p, img_center,rho)*(-1 if p[1]>img_center[1] else 1)
        y = math.sqrt((rho**2)-(x**2)-(z**2))
        return (x,y,z)


    def point_cylindrical_coords(self, p, distance):
        # The ISO standard 31-11 recommends (ρ, φ, z), where ρ is the radial
        # coordinate, φ the azimuth, and z the height.
        img_center = (self.IMAGE_WIDTH/2,self.IMAGE_HEIGHT/2)
        z = obj_height(p,img_center,distance)*(-1 if p[1] > img_center[1] else 1)
        phi = (p[0]/self.IMAGE_WIDTH-0.5)*self.HORIZONTAL_FOV
        rho = math.sqrt(distance**2-z**2)
        return (rho, phi, z)

    def roi_center(self, roi):
        return (roi[0]+roi[2]/2, roi[1]+roi[3]/2)

    def toggle_roi(self):
        self.show_roi = not self.show_roi

    def update_frame(self):
        current_frame = robot.sensors.camera.value
        if(self.show_image):
            if self.show_roi and len(self.current_rois) != 0 :
                frame_copy = current_frame.copy()
                for roi in self.current_rois:
                    cv2.rectangle(frame_copy, (roi[0],roi[1]),
                            (roi[0]+roi[2],roi[1]+roi[3]), (255, 0 , 0), 2 )
                    cv2.circle(frame_copy, roi_center(roi), 2, (0,255,0),-1)
                cv2.circle(frame_copy, (self.IMAGE_WIDTH/2, self.IMAGE_HEIGHT/2),2,(255,0,0),-1)
                cv2.imshow('feed', frame_copy)
            else:
                cv2.imshow('feed', current_frame)
            cv2.waitKey(1)
        return current_frame

    def scan(self):
        self.clear_saved_frames()
        rotation = 0
        for _ in range(7):
            rotation = rotation + robot.movement.rotate(57.29578,0.1)
            robot.vision.save_frame(self.update_frame())
        offset = rotation % 360
        rotation = rotation + robot.movement.rotate(-offset,0.1)
        return rotation

    def clear_saved_frames(self):
        self.current_frames = []

    def save_frame(self,frame):
        self.current_frames.append(frame)

