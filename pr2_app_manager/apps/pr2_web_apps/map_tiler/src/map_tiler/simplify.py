#!/usr/bin/env python

import rosweb

from nav_msgs.msg import Path

import math
import time

class Simplifier(rosweb.ROSWebSubTopic):
  def __init__(self, topic, factory, main_rwt):
    rosweb.ROSWebSubTopic.__init__(self, topic, factory, main_rwt)

  def simplify_points (self, pts, tolerance): 
      anchor  = 0
      floater = len(pts) - 1
      stack   = []
      keep    = set()

      stack.append((anchor, floater))  
      while stack:
          anchor, floater = stack.pop()
        
          # initialize line segment
          if pts[floater] != pts[anchor]:
              anchorX = float(pts[floater].pose.position.x - pts[anchor].pose.position.x)
              anchorY = float(pts[floater].pose.position.y - pts[anchor].pose.position.y)
              seg_len = math.sqrt(anchorX ** 2 + anchorY ** 2)
              # get the unit vector
              anchorX /= seg_len
              anchorY /= seg_len
          else:
              anchorX = anchorY = seg_len = 0.0
      
          # inner loop:
          max_dist = 0.0
          farthest = anchor + 1
          for i in range(anchor + 1, floater):
              dist_to_seg = 0.0
              # compare to anchor
              vecX = float(pts[i].pose.position.x - pts[anchor].pose.position.x)
              vecY = float(pts[i].pose.position.y - pts[anchor].pose.position.y)
              seg_len = math.sqrt( vecX ** 2 + vecY ** 2 )
              # dot product:
              proj = vecX * anchorX + vecY * anchorY
              if proj < 0.0:
                  dist_to_seg = seg_len
              else: 
                  # compare to floater
                  vecX = float(pts[i].pose.position.x - pts[floater].pose.position.x)
                  vecY = float(pts[i].pose.position.y - pts[floater].pose.position.y)
                  seg_len = math.sqrt( vecX ** 2 + vecY ** 2 )
                  # dot product:
                  proj = vecX * (-anchorX) + vecY * (-anchorY)
                  if proj < 0.0:
                      dist_to_seg = seg_len
                  else:  # calculate perpendicular distance to line (pythagorean theorem):
                      dist_to_seg = math.sqrt(abs(seg_len ** 2 - proj ** 2))
                  if max_dist < dist_to_seg:
                      max_dist = dist_to_seg
                      farthest = i

          if max_dist <= tolerance: # use line segment
              keep.add(anchor)
              keep.add(floater)
          else:
              stack.append((anchor, farthest))
              stack.append((farthest, floater))

      keep = list(keep)
      keep.sort()
      return [pts[i] for i in keep]

  def transform(self, msg):
    newmsg = Path()
    newmsg.poses = self.simplify_points(msg.poses, 0.05)
    return newmsg  
