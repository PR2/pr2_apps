#!/usr/bin/env python

import copy
import math
import rospy
import rosservice
import Image
import cStringIO
import unavail
import threading

import tf
import tf.transformations

from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, PoseStamped, Pose, Quaternion, Point
from nav_msgs.msg import MapMetaData, OccupancyGrid
from numpy import float64

from simplify import Simplifier
import rosweb

_lock = threading.Lock()

_map_cache = {}
_scale_cache = {}
_tile_cache = {}

def config_plugin(context):
  global goal_publisher, pose_publisher
  goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped)
  pose_publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)

  context.register_handler("/map", map_tiler_handler)
  context.register_subtopic("/move_base_node/NavfnROS/plan:simplified", Simplifier)

def trans(x):
  if x < 0:
    return chr(x + 256)
  else:
    return chr(x)

def get_map(service_name):
  map_key = service_name
  if not _map_cache.has_key(map_key):
    service_class = rosservice.get_service_class_by_name('/' + service_name)
    service_proxy = rospy.ServiceProxy(service_name, service_class)
    resp = service_proxy()
    size = (resp.map.info.width, resp.map.info.height)
    s = "".join([trans(x) for x in resp.map.data])
    im = Image.frombuffer('L', size, s, "raw", 'L', 0, -1)
    remap = {0:255, 100: 0, 255:128}
    im = im.point(lambda x: remap.get(x, 0))
    resolution = resp.map.info.resolution
    _map_cache[map_key] = (im, resolution)
  else:
     (im, resolution) = _map_cache[map_key]
  return (im, resolution)

def get_scaled_map(service_name, scale):
  scale_key = '%s:%s' % (service_name, scale)
  if not _scale_cache.has_key(scale_key):
    (im, resolution) = get_map(service_name)
    if scale != 1:
      size = im.size
      im = im.resize((int(size[0] / scale), int(size[1] / scale)), Image.ANTIALIAS)
    _scale_cache[scale_key] = im
  else:
    im = _scale_cache[scale_key]
  return im

def get_tile(service_name, scale, x, y, width, height):
  tile_key = '%s:%s:%s:%s:%s:%s' % (service_name, scale, x, y, width, height)
  if not _tile_cache.has_key(tile_key):
    # Get the map (possibly cached) from the map server
    im = get_scaled_map(service_name, scale)

    # Crop the requested tile and convert to JPEG
    im = im.crop((x, y, x + width, y + height))
    buf = cStringIO.StringIO()
    im.save(buf, format='JPEG')
    jpeg = buf.getvalue()
    _tile_cache[tile_key] = jpeg
  else:
    jpeg = _tile_cache[tile_key]
  return jpeg

def send_image(self, jpeg, cache_control='max-age=3600'):
  try:
    self.send_response(200)
    self.send_header('Cache-Control', cache_control)
    self.send_header('Content-Type', 'image/jpeg')
    self.send_header('Content-Length', str(len(jpeg)))
    self.end_headers()
    self.wfile.write(jpeg)
  except:
    pass

def get_tile_handler(self, path, qdict):
  service_name = qdict.get('service', ['static_map'])[0]
  scale = float(qdict.get('scale', [1])[0])
  x = int(qdict.get('x', [0])[0])
  y = int(qdict.get('y', [0])[0])
  width = int(qdict.get('width', [256])[0])
  height = int(qdict.get('height', [256])[0])

  try:
    jpeg = get_tile(service_name, scale, x, y, width, height)
  except Exception, reason:
    import base64
    jpeg = base64.decodestring(unavail.unavail)
  send_image(self, jpeg)

def get_pose(self, qdict):
  x = float(qdict.get('x', [0])[0])
  y = float(qdict.get('y', [0])[0])
  angle = float(qdict.get('angle', [0])[0])
  p = Point(x, y, 0)
  q = tf.transformations.quaternion_from_euler(0, 0, angle)
  q = Quaternion(q[0], q[1], q[2], q[3])
  return Pose(p, q)

def set_goal_handler(self, path, qdict):
  pose = get_pose(self, qdict)
  h = rospy.Header()
  h.stamp= rospy.get_rostime()
  h.frame_id = '/map'
  goal = PoseStamped(h, pose)
  goal_publisher.publish(goal)
  self.send_success()

def set_pose_handler(self, path, qdict):
  pose = get_pose(self, qdict)
  h = rospy.Header()
  h.stamp= rospy.get_rostime()
  h.frame_id = '/map'
  cov = [float64(0)] * 36
  cov[6*0+0] = 0.5 * 0.5;
  cov[6*1+1] = 0.5 * 0.5;
  cov[6*3+3] = math.pi/12.0 * math.pi/12.0;
  pose_publisher.publish(PoseWithCovarianceStamped(h, PoseWithCovariance(pose, cov)))
  self.send_success()

def get_extents_handler(self, path, qdict):
  service_name = qdict.get('service', ['static_map'])[0]
  callback = qdict.get("callback", [''])[0]
  (im, resolution) = get_map(service_name)
  buf = cStringIO.StringIO()
  buf.write('{')
  buf.write('"width": %d,' % im.size[0])
  buf.write('"height": %d,' % im.size[1])
  buf.write('"resolution": %s,' % resolution)
  buf.write('}')
  buf = buf.getvalue()
  self.send_success(buf, callback)

def map_tiler_handler(self, path, qdict):
  try:
    _lock.acquire()

    if   path == '/map/get_tile':    get_tile_handler(self, path, qdict)
    elif path == '/map/get_extents': get_extents_handler(self, path, qdict)
    elif path == '/map/set_goal':    set_goal_handler(self, path, qdict)
    elif path == '/map/set_pose':    set_pose_handler(self, path, qdict)
    else:
      return False
    return True
  finally:
    _lock.release()
