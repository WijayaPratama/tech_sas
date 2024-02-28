#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

def callback_odometry(msg):
    # Fungsi callback untuk menangani pesan nav_msgs/Odometry

    # Mengakses informasi posisi dari pesan Odometry
    posisi = msg.pose.pose.position
    orientasi = msg.pose.pose.orientation

    # Mengakses kecepatan linear dan angular
    kecepatan_linear = msg.twist.twist.linear
    kecepatan_angular = msg.twist.twist.angular

    # Lakukan operasi yang diinginkan dengan informasi posisi dan kecepatan
    print("Menerima Odometry:")
    print("Posisi: x={}, y={}, z={}".format(posisi.x, posisi.y, posisi.z))
    # print("Orientasi: x={}, y={}, z={}, w={}".format(orientasi.x, orientasi.y, orientasi.z, orientasi.w))
    # print("Kecepatan Linear: x={}, y={}, z={}".format(kecepatan_linear.x, kecepatan_linear.y, kecepatan_linear.z))
    # print("Kecepatan Angular: x={}, y={}, z={}".format(kecepatan_angular.x, kecepatan_angular.y, kecepatan_angular.z))

if __name__ == '__main__':
    rospy.init_node('pemantau_odometry', anonymous=True)

    # Tentukan topik yang ingin Anda langgani
    nama_topik = "/camera/odom/sample"

    # Persiapkan pelanggan dengan fungsi panggilan kembali yang ditentukan
    rospy.Subscriber(nama_topik, Odometry, callback_odometry)

    # Biarkan skrip berjalan terus
    rospy.spin()
