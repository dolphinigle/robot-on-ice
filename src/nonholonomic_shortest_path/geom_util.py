import math

from shapely.geometry.linestring import LineString
from shapely.geometry.point import Point


CLOCKWISE = 0
COUNTER_CLOCKWISE = 1
EPS = 1e-9
ANGLE_MOD = math.pi * 2.0


class Circle(object):
  def __init__(self, center, radius):
    self.center = center
    self.radius = radius

  def PointToAngle(self, point):
    '''Inverse of below.'''
    return math.atan2(point[1] - self.center[1],
                      point[0] - self.center[0]) % (math.pi * 2.0)

  def AngleToPoint(self, angle):
    # Returns the Point on the specific angle on this circle/arc.
    return ((self.center[0] + math.cos(angle) * self.radius,
             self.center[1] + math.sin(angle) * self.radius))


class Arc(Circle):
  # Arc of a circle.
  def __init__(self, center, radius, begin_radian, end_radian, direction, circle_pk=None):
    '''
      radian is ccw wrt positive x axis.
      Since clockwise is basically the same if we swap begin and end,
      we have an evilness called _begin_radian and _end_radian. For the purpose
      of calculation these values should be used instead.
    '''
    super(Arc, self).__init__(center, radius)

    assert direction in [CLOCKWISE, COUNTER_CLOCKWISE]
    assert radius >= 0
    assert 0 <= begin_radian <= 2.0 * math.pi
    assert 0 <= end_radian <= 2.0 * math.pi
    self.center = center
    self.radius = radius
    self.begin_radian = begin_radian
    self.end_radian = end_radian
    self.circle_pk = circle_pk

    if direction == CLOCKWISE:
      begin_radian, end_radian = end_radian, begin_radian
    self._begin_radian = begin_radian
    self._end_radian = end_radian

    self.direction = direction

  def Length(self):
    delta = (self._end_radian - self._begin_radian) % ANGLE_MOD
    return delta * self.radius

  def MidAngle(self, distance=0.5):
    # Returns the middle angle between begin and end.
    # distance is distance from first point in fraction. So, 0.25 will give the first quarter point.
    assert 0.0 <= distance <= 1.0
    delta = (self._end_radian - self._begin_radian) % (2.0 * math.pi)
    if self.direction == CLOCKWISE:
      distance = 1.0 - distance
    angle = (self._begin_radian + delta * distance) % (2.0 * math.pi)
    return angle

  def IsPointOnArcOrOnArcBoundary(self, point):
    if abs((point[0] - self.center[0])**2 + (point[1] - self.center[1])**2 - self.radius**2) > EPS:
      return False
    angle = self.PointToAngle(point)
    delta_all = (self._end_radian - self._begin_radian) % (2.0 * math.pi)
    delta = (angle - self._begin_radian) % (2.0 * math.pi)
    return delta <= delta_all


def IsCcw(line_string):
  val = 0
  for p1, p2 in zip(line_string.coords, line_string.coords[1:] + line_string.coords[:1]):
    val += p1[0] * p2[1] - p1[1] * p2[0]
  return val >= 0


def ShortestToLine(line, point):
  point_a = line.coords[0]
  point_b = line.coords[1]
  dividend = (point_b[0] - point_a[0])**2 + (point_b[1] - point_a[1])**2
  proj = ((point[0] - point_a[0]) * (point_b[0] - point_a[0]) + (point[1] - point_a[1]) * (point_b[1] - point_a[1])) / dividend
  cpx = point_a[0] + proj * (point_b[0] - point_a[0])
  cpy = point_a[1] + proj * (point_b[1] - point_a[1])
  return (cpx, cpy)


def LineSegmentIntersectStrict(line1, line2):
  cp1 = CrossProd(line1[0], line1[1], line2[0])
  cp2 = CrossProd(line1[0], line1[1], line2[1])
  cp3 = CrossProd(line2[0], line2[1], line1[0])
  cp4 = CrossProd(line2[0], line2[1], line1[1])
  if abs(cp1) <= EPS or abs(cp2) <= EPS or abs(cp3) <= EPS or abs(cp4) <= EPS:
    return False
  return cp1 * cp2 < 0 and cp3 * cp4 < 0


def CircleLineSegmentIntersections(circle, segment):
  center = Point(circle.center[0], circle.center[1])
  distance = segment.distance(center)
  if distance > circle.radius + EPS:
    return []

  super_point = ShortestToLine(segment, circle.center)
  super_point_p = Point(super_point[0], super_point[1])

  orig_point = segment.coords[0]
  dest_point = segment.coords[1]

  dist = Point(orig_point[0], orig_point[1]).distance(super_point_p)
  if ((dest_point[0] - orig_point[0]) * (orig_point[0] - super_point[0]) >= 0 and
      (dest_point[1] - orig_point[1]) * (orig_point[1] - super_point[1]) >= 0):
    dist *= -1

  distance = center.distance(super_point_p)
  if distance > circle.radius + EPS:
    return []

  val = circle.radius**2 - distance**2
  if val < 0:
    val = 0
  front_side = math.sqrt(val)
  intersections = []
  for i in range(-1, 2, 2):
    if i == 1 and front_side < EPS:
      break

    new_dist = dist + front_side * i
    if new_dist < -EPS or new_dist > segment.length + EPS:
      continue
    intersection_point = segment.interpolate(new_dist)
    intersections.append([intersection_point.x, intersection_point.y])
  return intersections


def CrossProd(p1, p2, p3):
  return p1[0] * p2[1] + p2[0] * p3[1] + p3[0] * p1[1] - p1[1] * p2[0] - p2[1] * p3[0] - p3[1] * p1[0]


def CirclePolygonIntersections(circle, polygon):
  '''Returns all points of intersections between circle and polygon'''
  intersections = []
  for coords in list(map(lambda interior: interior.coords, polygon.interiors)) + [polygon.exterior.coords]:
    for p1, p2 in zip(coords, coords[1:]):
      intersections.extend(CircleLineSegmentIntersections(circle, LineString((p1, p2))))

  return intersections


def IsArcStrictIntersectPoly(arc, poly):
  # First check if two points in it is in poly.
  # Touching is OK.
  if poly.contains(Point(*arc.AngleToPoint(arc.MidAngle()))):
    return True

  # Then, check if arc intersect any of the poly's boundaries.
  intersections = CirclePolygonIntersections(arc, poly)

  # Now, classify the intersections that belongs to the arcs (including boundaries)
  angles = []
  for intersection in intersections:
    if arc.IsPointOnArcOrOnArcBoundary(intersection):
      angles.append(arc.PointToAngle(intersection))

  angles.append(arc._begin_radian)
  angles.append(arc._end_radian)

  angles.sort(key=lambda angle: (angle - arc._begin_radian) % (math.pi * 2.0))

  for p1, p2 in zip(angles, angles[1:]):
    if abs(p1 - p2) <= EPS:
      continue
    dist = (p2 - p1) % (math.pi * 2.0)
    midangle = (p1 + dist / 2.0) % (math.pi * 2.0)
    point = arc.AngleToPoint(midangle)
    if poly.contains(Point(point[0], point[1])):
      return True

  return False


def GetTangentLinePoint(circle1, is_circle1_ccw, point):
  '''Returns the angle that make the tangent with point.'''
  angle = math.atan2(point[1] - circle1.center[1],
                     point[0] - circle1.center[0]) % ANGLE_MOD

  distance = math.sqrt((circle1.center[0] - point[0])**2 +
                       (circle1.center[1] - point[1])**2)
  tangent = math.sqrt(distance**2 - circle1.radius**2)
  tangent_angle = math.atan2(tangent, circle1.radius) % ANGLE_MOD

  if is_circle1_ccw:
    return (angle - tangent_angle) % ANGLE_MOD
  else:
    return (angle + tangent_angle) % ANGLE_MOD


def GetTangentLine(circle1, is_circle1_ccw, circle2, is_circle2_ccw):
  '''Returns the pair of angles that make the tangent. Assume the two circles
  are equi-radius.'''
  assert(abs(circle1.radius - circle2.radius) <= EPS)

  angle = math.atan2(circle2.center[1] - circle1.center[1],
                     circle2.center[0] - circle1.center[0]) % ANGLE_MOD
  if is_circle1_ccw and is_circle2_ccw:
    angle1 = angle2 = (angle - math.pi / 2.0) % ANGLE_MOD
  elif not is_circle1_ccw and not is_circle2_ccw:
    angle1 = angle2 = (math.pi / 2.0 + angle) % ANGLE_MOD
  else:
    distance = math.sqrt((circle1.center[0] - circle2.center[0])**2 +
                         (circle1.center[1] - circle2.center[1])**2)
    tangent = math.sqrt(distance**2 - (2.0 * circle1.radius)**2)
    tangent_angle = math.atan2(tangent, 2.0 * circle1.radius) % ANGLE_MOD
    if is_circle1_ccw:
      angle1 = (angle - tangent_angle) % ANGLE_MOD
      angle2 = (angle1 + math.pi) % ANGLE_MOD
    else:
      angle1 = (angle + tangent_angle) % ANGLE_MOD
      angle2 = (angle1 + math.pi) % ANGLE_MOD
  return (angle1, angle2)


def CircleIntersects(circle1, circle2):
  return (circle1.radius + circle2.radius)**2 > (circle1.center[0] - circle2.center[0])**2 + (circle1.center[1] - circle2.center[1])**2


def IsAngleBetween(angle, lb, ub):
  if ub < lb:
    ub += ANGLE_MOD
  if angle < lb:
    angle += ANGLE_MOD
  return lb + EPS < angle < ub - EPS


def AngleAlmostEqual(angle1, angle2):
  if abs(angle1 - angle2) < EPS:
    return True
  if abs(abs(angle1 - angle2) - ANGLE_MOD) < EPS:
    return True
  return False


def AngleShortestDiff(angle1, angle2):
  if AngleAlmostEqual(angle1, angle2):
    return 0.0
  return min((angle1 - angle2) % ANGLE_MOD,
             (angle2 - angle1) % ANGLE_MOD)

