# Synthetic data: [1.2716150429967599, -1.477156828124997], 
# [2.1581718914246473, -32.273638578124995], [5.295426423205694, -41.0694360546875], 
# [1.3674836259322483, -99.55141775000001]

import pfilter
from pfilter import ParticleFilter, gaussian_noise, squared_error, independent_sample
from scipy.stats import norm, gamma, uniform 
columns = ["x", "y"]

# prior sampling function for each variable
# (assumes x and y are coordinates in the range 0-1000)    
prior_fn = independent_sample([uniform(loc=0, scale=1000).rvs, uniform(loc=0, scale=1000).rvs])

# Debug
for i in prior_fn:
    print(i)
print(prior_fn)

# create the filter
#pf = pfilter.ParticleFilter( prior_fn=prior_fn, observe_fn=blob, n_particles=200, dynamics_fn=velocity, noise_fn=lambda x: 
#gaussian_noise(x, sigmas=[0.2, 0.2]), weight_fn=lambda x,y:squared_error(x, y, sigma=2), resample_proportion=0.1, column_names = columns)
            
# assuming image of the same dimensions/type as blob will produce
#pf.update(image) 