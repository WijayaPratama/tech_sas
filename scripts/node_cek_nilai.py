#!/usr/bin/env python3

import rospy
from robotic_sas_auv_ros.msg import Orientation

def callback_odometry(msg):
    # print(msg)
    # Fungsi callback untuk menangani pesan msg/Orientation

    # Mengakses informasi posisi dari pesan Orientation
    # orientasi = msg.orientation
    print("Posisi: x={}, y={}, z={}, w={}".format(msg.x, msg.y, msg.z, msg.w))


    # Mengakses kecepatan linear dan angular
    # kecepatan_linear = msg.twist.twist.linear
    # kecepatan_angular = msg.twist.twist.angular

    # Lakukan operasi yang diinginkan dengan informasi posisi dan kecepatan
    # print("Menerima Orientasi:")
    # print("Posisi: x={}, y={}, z={}".format(orientasi.x,orientasi.y,orientasi.z))
    # print("Orientasi: x={}, y={}, z={}, w={}".format(orientasi.x, orientasi.y, orientasi.z, orientasi.w))
    # print("Kecepatan Linear: x={}, y={}, z={}".format(kecepatan_linear.x, kecepatan_linear.y, kecepatan_linear.z))
    # print("Kecepatan Angular: x={}, y={}, z={}".format(kecepatan_angular.x, kecepatan_angular.y, kecepatan_angular.z))

if __name__ == '__main__':
    rospy.init_node('pemantau_orientasi', anonymous=True)

    # Tentukan topik yang ingin Anda langgani
    nama_topik = "/orientation"

    # Persiapkan pelanggan dengan fungsi panggilan kembali yang ditentukan
    rospy.Subscriber(nama_topik, Orientation, callback_odometry)

    # Biarkan skrip berjalan terus
    rospy.spin()