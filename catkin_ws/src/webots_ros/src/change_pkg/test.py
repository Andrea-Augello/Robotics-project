import pfilter
import numpy as np
from scipy.stats import norm, gamma, uniform 
# testing only
import skimage.draw
import cv2
from scipy.stats import norm, gamma, uniform 
from pfilter import ParticleFilter, gaussian_noise, squared_error, independent_sample


def blob(x):
    """Given an Nx3 matrix of blob positions and size, 
    create N 32x32 images, each with a blob drawn on 
    them given by the value in each row of x
    
    One row of x = [x,y,radius]."""
    y = np.zeros((x.shape[0], 32, 32))
    for i,particle in enumerate(x):
        rr,cc = skimage.draw.circle(particle[0], particle[1], 
                                    particle[2], shape=(32,32))
        y[i,rr,cc] = 1
    return y

columns = ["x", "y", "radius", "dx", "dy"]

# prior sampling function for each variable
# (assumes x and y are coordinates in the range 0-32)    
prior_fn = independent_sample([uniform(loc=0, scale=32).rvs, 
            uniform(loc=0, scale=32).rvs, 
            gamma(a=2,loc=0,scale=10).rvs,
            norm(loc=0, scale=0.5).rvs,
            norm(loc=0, scale=0.5).rvs])
                                    
# very simple linear dynamics: x += dx
def velocity(x):
    xp = np.array(x)
    xp[0:2] += xp[3:5]        
    return xp

# create the filter
pf = pfilter.ParticleFilter(
prior_fn=prior_fn, 
observe_fn=blob,
n_particles=200,
dynamics_fn=velocity,
noise_fn=lambda x: 
gaussian_noise(x, sigmas=[0.2, 0.2, 0.1, 0.05, 0.05]),
weight_fn=lambda x,y:squared_error(x, y, sigma=2),
resample_proportion=0.1,
column_names = columns)

# start in centre, random radius
x,y,s = 16,16,np.random.uniform(2,8)
# random movement direction
dx = -0.2
#dx = np.random.uniform(-0.1,0.1)
dy = np.random.uniform(-0.1,0.1)    
cv2.namedWindow('img',cv2.WINDOW_NORMAL)
cv2.namedWindow('samples',cv2.WINDOW_NORMAL)
cv2.resizeWindow('img', 320,320)
cv2.resizeWindow('samples', 320,320)
for i in range(200):        
    img = blob(np.array([[x,y,s]]))
    pf.update(img)
    cv2.imshow("img", np.squeeze(img))
    color = cv2.cvtColor(pf.mean_hypothesis.astype(np.float32), cv2.COLOR_GRAY2RGB)
    x_hat,y_hat,s_hat,dx_hat,dy_hat = pf.mean_state
    
    # x,y exchange because of ordering between skimage and opencv
    cv2.circle(color, (int(y_hat), int(x_hat)),
                int(s_hat), (0,1,0), 1)
    
    cv2.line(color, (int(y_hat), int(x_hat)),
                    (int(y_hat+dy_hat*5), int(x_hat+dx_hat*5)),
                    (0,0,1))
    
    cv2.imshow("samples", color)        
    cv2.waitKey(20)
    x+=dx
    y+=dy
    
cv2.destroyAllWindows()
test_filter()