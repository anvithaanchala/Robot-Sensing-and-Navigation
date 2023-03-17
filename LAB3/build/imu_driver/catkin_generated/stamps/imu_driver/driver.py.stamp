from imu_driver.msg import Vectornav
from imu_driver.srv import convert_to_quaternion, convert_to_quaternionResponse
import serial
import rospy
import utm
from std_msgs.msg import String
import sys
import time
import math

def talker():
    rospy.init_node('talker')
    port = rospy.get_param("~port_number")
    rate = rospy.Rate(40)
    ser = serial.Serial(port=port, baudrate=115200, bytesize=8, timeout=5, stopbits=serial.STOPBITS_ONE)
    ser.write(b"VNWRG,07,40*XX")
    pub = rospy.Publisher('imu', Vectornav, queue_size=10)
    now = rospy.get_rostime()

    # Wait for the service to become available
    rospy.wait_for_service('/convert_to_quaternion')
    quat_client = rospy.ServiceProxy('/convert_to_quaternion', convert_to_quaternion)
    
    while not rospy.is_shutdown():
        sentence1 = ser.readline()
        sentence = sentence1.decode("utf-8")
        if '$VNYMR' in (sentence):
            print(sentence)
            elements = sentence.split(',')
            roll = float(elements[3])
            pitch = float(elements[2])
            yaw = float(elements[1])

            try:
                # Call the service client with roll, pitch, and yaw to get the quaternion response
                resp = quat_client(roll, pitch, yaw)
                qx = resp.x
                qy = resp.y
                qz = resp.z
                qw = resp.w
            except rospy.ServiceException as e:
                # Log any errors that occur with the service client
                rospy.logerr("Quaternion conversion service call failed: %s", e)
                continue

            msg = Vectornav()
            msg.imu.orientation.x = qx
            msg.imu.orientation.y = qy
            msg.imu.orientation.z = qz
            msg.imu.orientation.w = qw

            angl_z1 = elements[12]
            angl_z = float(angl_z1[:-8])

            msg.imu.angular_velocity.x = float(elements[10])
            msg.imu.angular_velocity.y = float(elements[11])
            msg.imu.angular_velocity.z = angl_z

            msg.imu.linear_acceleration.x = float(elements[7])
            msg.imu.linear_acceleration.y = float(elements[8])
            msg.imu.linear_acceleration.z = float(elements[9])

            msg.mag_field.magnetic_field.x = float(elements[4])
            msg.mag_field.magnetic_field.y = float(elements[5])
            msg.mag_field.magnetic_field.z = float(elements[6])

            # Set the message header timestamp
            msg.Header.stamp.secs = rospy.Time.now()
            msg.Header.frame_id= "imu1_frame"
            rospy.loginfo("Current system time: {}".format(rospy.Time.now()))
            pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
       pass

