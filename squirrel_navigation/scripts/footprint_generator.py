# footprint_generator.py --- 
# 
# Filename: footprint_generator.py
# Description: Generate footprint for robot
# Author: Federico Boniardi
# Maintainer: boniardi@informatik.uni-freiburg.de
# Created: Mo Jan 18 15:58:05 2016 (+0100)
# Version: 
# Last-Updated: 
#           By: 
#     Update #: 0
# URL: 
# Keywords: 
# Compatibility: 
# 
# 

# Code:

import math
import sys

def generate_footprint_(inner_radius, outer_radius, n, alpha_max, alpha_min):
    footprint = []
    a_max = math.fmod(alpha_max,2*math.pi)
    a_min = math.fmod(alpha_min,2*math.pi)
    da = 2*math.pi/(n)
    for i in range(int(n)):
        a = da*i
        if (a<=a_min or a>=a_max):
            footprint += [[outer_radius*math.cos(a),outer_radius*math.sin(a)]]
        else:
            footprint += [[inner_radius*math.cos(a),inner_radius*math.sin(a)]]
    return footprint

if __name__ == '__main__':
    inner_radius, outer_radius,n,alpha_max,alpha_min = [float(s) for s in sys.argv[1:]] 
    print generate_footprint_(inner_radius, outer_radius,n,alpha_max,alpha_min)



# 
# footprint_generator.py ends here
