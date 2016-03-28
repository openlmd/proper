#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class ShapeMarker():
    def __init__(self, frame_id='/world'):
        self.marker = Marker()
        self.marker.lifetime = rospy.Duration()

        self.set_frame(frame_id)

        self.set_scale()
        self.set_color()
        self.set_position()
        self.set_orientation()

    def set_frame(self, frame_id):
        self.marker.header.frame_id = frame_id

    def set_color(self, color=(1, 1, 1, 1)):
        self.marker.color.r = color[0]
        self.marker.color.g = color[1]
        self.marker.color.b = color[2]
        self.marker.color.a = color[3]

    def set_scale(self, scale=(1, 1, 1)):
        self.marker.scale.x = scale[0]
        self.marker.scale.y = scale[1]
        self.marker.scale.z = scale[2]

    def set_position(self, position=(0, 0, 0)):
        self.marker.pose.position.x = position[0]
        self.marker.pose.position.y = position[1]
        self.marker.pose.position.z = position[2]

    def set_orientation(self, orientation=(0, 0, 0, 1)):
        self.marker.pose.orientation.x = orientation[0]
        self.marker.pose.orientation.y = orientation[1]
        self.marker.pose.orientation.z = orientation[2]
        self.marker.pose.orientation.w = orientation[3]


class ArrowMarker(ShapeMarker):
    def __init__(self, length):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.ARROW
        self.set_length(length)

    def set_length(self, length):
        self.set_scale((length, 0.1 * length, 0.1 * length))


class CubeMarker(ShapeMarker):
    def __init__(self):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.CUBE


class SphereMarker(ShapeMarker):
    def __init__(self):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.SPHERE


class CylinderMarker(ShapeMarker):
    def __init__(self):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.CYLINDER


class LinesMarker(ShapeMarker):
    def __init__(self):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.LINE_STRIP  #self.marker.LINE_LIST
        self.marker.points = [Point(0, 0, 0)]
        self.set_size()

    def set_size(self, size=0.001):
        self.set_scale(scale=(size, size, 1.0))

    def set_points(self, points):
        self.marker.points = [Point(x, y, z) for x, y, z in points]


class PointsMarker(ShapeMarker):
    def __init__(self):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.POINTS
        self.marker.points = [Point(0, 0, 0),
                              Point(1, 0, 0),
                              Point(1, 1, 0)]
        self.set_size()

    def set_size(self, size=0.025):
        self.set_scale(scale=(size, size, 1.0))

    def set_points(self, points):
        self.marker.points = [Point(x, y, z) for x, y, z in points]


class TextMarker(ShapeMarker):
    def __init__(self, text):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.TEXT_VIEW_FACING
        self.set_text(text)
        self.set_size()

    def set_text(self, text):
        self.marker.text = text

    def set_size(self, size=0.1):
        self.set_scale(scale=(1.0, 1.0, size))


class MeshMarker(ShapeMarker):
    def __init__(self, mesh_resource="package://etna_triangulation/meshes/robot.dae", frame_id='/world'):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.MESH_RESOURCE
        self.marker.mesh_resource = mesh_resource
        self.marker.mesh_use_embedded_materials = True


class TriangleListMarker(ShapeMarker):
    def __init__(self):
        ShapeMarker.__init__(self)
        self.marker.type = self.marker.TRIANGLE_LIST
        self.marker.points = [Point(0, 0, 0), Point(1, 0, 0), Point(1, 1, 0),
                              Point(0, 0, 0), Point(1, 0, 0), Point(1, 0, 1)]
        #self.set_size()

    def set_points(self, points):
        self.marker.points = [Point(x, y, z) for x, y, z in points]


class PartMarkers():
    def __init__(self):
        marker_array = MarkerArray()

        # marker_array.markers.append(mesh_marker.marker)
        # marker_array.markers.append(ArrowMarker(1).marker)
        # marker_array.markers.append(CubeMarker().marker)
        # marker_array.markers.append(CylinderMarker().marker)
        # marker_array.markers.append(SphereMarker().marker)
        # marker_array.markers.append(LinesMarker().marker)
        # marker_array.markers.append(PointsMarker().marker)



if __name__ == '__main__':
    import numpy as np

    rospy.init_node("markers")

    pub_marker = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    pub_marker_array = rospy.Publisher(
        'visualization_marker_array', MarkerArray, queue_size=10)

    marker_array = MarkerArray()

    #mesh_marker = MeshMarker(mesh_resource="package://etna_triangulation/meshes/test.dae")
    mesh_marker = TriangleListMarker()
    mesh_marker.set_color(color=(1.0, 0.5, 0.0, 0.75))

    points = np.array([[0.0, 0.0, 0.0],
                       [0.1, 0.0, 0.0],
                       [0.1, 0.1, 0.0],
                       [0.1, 0.1, 0.1],
                       [0.0, 0.1, 0.1],
                       [0.0, 0.0, 0.1],
                       [0.0, 0.0, 0.0]])

    cube = CubeMarker()
    cube.set_scale((0.1, 0.1, 0.1))
    cube.set_color((1, 1, 1, 0.7))
    cube.set_frame('/workobject')
    marker_array.markers.append(cube.marker)
    lines = LinesMarker()
    lines.set_points(points)
    lines.set_color((1, 0, 0, 1))
    lines.set_frame('/workobject')
    marker_array.markers.append(lines.marker)
    arrow = ArrowMarker(0.1)
    arrow.set_color((0, 0, 1, 1))
    arrow.set_frame('/workobject')
    arrow.set_position((0.2, 0.2, 0.2))
    arrow.set_orientation((0, 0, 0, 1))
    marker_array.markers.append(arrow.marker)

    # Renumber the marker IDs
    for id, m in enumerate(marker_array.markers):
        m.id = id

    k = 0
    while not rospy.is_shutdown():
        pub_marker.publish(mesh_marker.marker)
        pub_marker_array.publish(marker_array)
        k = k + 0.01
        cube.set_position((k, 0, 0))
        rospy.sleep(1.0)


    # count = 0
    # MARKERS_MAX = 100
    #
    # while not rospy.is_shutdown():
    #    marker = Marker()
    #    marker.header.frame_id = "/world"
    #    marker.type = marker.SPHERE
    #    marker.action = marker.ADD
    #    marker.scale.x = 0.2
    #    marker.scale.y = 0.2
    #    marker.scale.z = 0.2
    #    marker.color.a = 1.0
    #    marker.color.r = 1.0
    #    marker.color.g = 1.0
    #    marker.color.b = 0.0
    #    marker.pose.orientation.w = 1.0
    #    marker.pose.position.x = math.cos(count / 50.0)
    #    marker.pose.position.y = math.cos(count / 40.0)
    #    marker.pose.position.z = math.cos(count / 30.0)
    #
    #    # We add the new marker to the MarkerArray, removing the oldest
    #    # marker from it when necessary
    #    if(count > MARKERS_MAX):
    #        markerArray.markers.pop(0)
    #
    #    markerArray.markers.append(marker)
    #
    #    # Renumber the marker IDs
    #   for id, m in enumerate(marker_array.markers):
    #       m.id = id
