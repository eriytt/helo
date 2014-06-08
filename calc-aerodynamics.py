#!/usr/bin/env python

from optparse import OptionParser
import sys
import math
from math import sqrt

options = ["mass", "thrust", "wingspan", "wingarea", "climbrate", "maxspeed",
           "cruisespeed", "stallspeed"]
parser = OptionParser()
for o in options:
    parser.add_option("--" + o)

(opts, args) = parser.parse_args(sys.argv)


def check_get_arg(argname):
    val = getattr(opts, argname)
    if val == None:
        print >> sys.stderr, "Please supply --%s" % argname
        sys.exit(-1)
    return float(val)

wingspan = check_get_arg("wingspan")
thrust = check_get_arg("thrust")
mass = check_get_arg("mass")
wingarea = check_get_arg("wingarea")
climbrate = check_get_arg("climbrate")
maxspeed = check_get_arg("maxspeed") / 3.6
cruisespeed = check_get_arg("cruisespeed") / 3.6
stallspeed = check_get_arg("stallspeed") / 3.6
air_density = 1.2041
g = 9.81


AR = pow(wingspan, 2.0) / wingarea
print "AR =", AR
e0 = 1.78 * (1.0 - (0.0045*pow(AR, 0.68))) - 0.64
K = 1.0 / (math.pi * AR * e0)
CL_max = (air_density * wingarea * pow(stallspeed, 2)) / (2.0 * mass * g)
CD0 = 2 * (mass * g) * K / (air_density * pow(cruisespeed, 2) * wingarea)
CLa = CL_max / 0.108
wing_angle =  (2.0 * mass * g) / (air_density * wingarea *pow(cruisespeed, 2.0)) / 0.108
print """
CD0    = %f
K      = %f
CLmax  = %f
CLa    = %f
wing a = %f
""" % (CD0, K, CL_max, CLa, wing_angle)
