#!/usr/bin/python3

import sys
import trimesh
import rospy
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Point
from nuc.msg import MeshWithFrame

if __name__ == '__main__':
    
    if len(sys.argv) != 2:
        print("Usage: demo.py <stl_file_path>")
        sys.exit(1)

    rospy.init_node("nuc_demo", anonymous=True)

    mesh_publisher = rospy.Publisher('/nuc_mesh', MeshWithFrame, queue_size=10)

    filename = sys.argv[1]

    mesh = trimesh.load_mesh(filename)

    mesh_msg = MeshWithFrame()

    for vertex in mesh.vertices:
        point = Point()
        point.x = vertex[0]
        point.y = vertex[1]
        point.z = vertex[2]
        mesh_msg.nuc_mesh.vertices.append(point)

    for face in mesh.faces:
        tri = MeshTriangle()
        tri.vertex_indices[0] = face[0]
        tri.vertex_indices[1] = face[1]
        tri.vertex_indices[2] = face[2]
        mesh_msg.nuc_mesh.triangles.append(tri)

    mesh_msg.nuc_frame = "nuc_demo_test_frame"

    rate = rospy.Rate(1)  # Publish at 10 Hz

    while not rospy.is_shutdown():
        mesh_publisher.publish(mesh_msg)
        rate.sleep()
