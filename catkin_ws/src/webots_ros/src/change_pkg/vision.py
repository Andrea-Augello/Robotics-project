from change_pkg.ros_interface import *
import cv2
import math
FOCAL_LENGTH   = 0.0036
IMAGE_HEIGHT   = 480
IMAGE_WIDTH    = 640
SENSOR_HEIGHT  = 0.002549874    # Rough guess, to be refined
SENSOR_HEIGHT  = 0.003399832    # Rough guess, to be refined
VERTICAL_FOV   = 44.56046121921962
HORIZONTAL_FOV = 57.29578

current_frames = []
current_rois = []
show_roi = False
show_image = True

def obj_dist_w(p1, p2, real_width):
    diff = (p1[0]-p2[0],p1[1]-p2[1])
    pixel_height = math.sqrt(diff[1]**2 + diff[1]**2)
    return (real_width*FOCAL_LENGTH*IMAGE_WIDTH)\
            /(pixel_height*SENSOR_WIDTH)

def obj_dist_h(p1, p2, real_height):
    diff = (p1[0]-p2[0],p1[1]-p2[1])
    pixel_height = math.sqrt(diff[0]**2 + diff[1]**2)
    return (real_height*FOCAL_LENGTH*IMAGE_HEIGHT)\
            /(pixel_height*SENSOR_HEIGHT)


def obj_height(p1, p2, real_distance):
    diff = (p1[0]-p2[0],p1[1]-p2[1])
    pixel_height = abs(diff[1])
    return (real_distance*pixel_height*SENSOR_HEIGHT)\
            /(FOCAL_LENGTH*IMAGE_HEIGHT)


def obj_width(p1, p2, real_distance):
    diff = (p1[0]-p2[0],p1[1]-p2[1])
    pixel_height = abs(diff[0])
    return (real_distance*pixel_height*SENSOR_WIDTH)\
            /(FOCAL_LENGTH*IMAGE_WIDTH)


def point_polar_coords(p, rho):
    # ISO 80000-2:2019 convention: radial distance r, polar angle θ
    # (theta), and azimuthal angle φ (phi). The symbol ρ (rho) is often
    # used instead of r. Phi here is the elevation angle (i.e. measured
    # from the reference plane), not the inclination angle (measured from
    # the Z axis).
    theta = (p[0]/IMAGE_WIDTH-0.5)*HORIZONTAL_FOV
    phi = -(p[1]/IMAGE_HEIGHT-0.5)*VERTICAL_FOV
    return (rho, theta, phi)


def point_coords(p, rho):
    img_center = (IMAGE_WIDTH/2,IMAGE_HEIGHT/2)
    x = obj_width( p, img_center,rho)*(-1 if p[0]<img_center[0] else 1)
    z = obj_height(p, img_center,rho)*(-1 if p[1]>img_center[1] else 1)
    y = math.sqrt((rho**2)-(x**2)-(z**2))
    return (x,y,z)


def point_cylindrical_coords(p, distance):
    # The ISO standard 31-11 recommends (ρ, φ, z), where ρ is the radial
    # coordinate, φ the azimuth, and z the height.
    img_center = (IMAGE_WIDTH/2,IMAGE_HEIGHT/2)
    z = obj_height(p,img_center,distance)*(-1 if p[1] > img_center[1] else 1)
    phi = (p[0]/IMAGE_WIDTH-0.5)*HORIZONTAL_FOV
    rho = math.sqrt(distance**2-z**2)
    return (rho, phi, z)

def roi_center(roi):
	return (roi[0]+roi[2]/2, roi[1]+roi[3]/2)

def toggle_roi():
    show_roi = not show_roi

def update_frame():
    current_frame = get_image()
    if(show_image):
        if show_roi and len(current_rois) != 0 :
            frame_copy = current_frame.copy()
            for roi in current_rois:
                cv2.rectangle(frame_copy, (roi[0],roi[1]),
                        (roi[0]+roi[2],roi[1]+roi[3]), (255, 0 , 0), 2 )
                cv2.circle(frame_copy, roi_center(roi), 2, (0,255,0),-1)
            cv2.circle(frame_copy, (IMAGE_WIDTH/2, IMAGE_HEIGHT/2),2,(255,0,0),-1)
            cv2.imshow('feed', frame_copy)
        else:
            cv2.imshow('feed', current_frame)
        cv2.waitKey(1)
    return current_frame

def clear_saved_frames():
    global current_frames
    current_frames = []

def save_frame(frame):
    global current_frames
    current_frames.append(frame)
