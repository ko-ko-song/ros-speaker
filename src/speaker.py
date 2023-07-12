#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from pygame import mixer
import rospkg



class Speaker:
    def __init__(self):
        rospy.init_node('speaker', anonymous=True)
        mixer.init()
        rospy.Subscriber("speaker", String, self.playSoundCallback)

        # Get the path to the current package
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('speaker')

        # Define the sound file paths
        self.sound_path = package_path + "/sounds/"

    def playSoundCallback(self, data):
        rospy.loginfo(f"Received on : {data.data}")
        if mixer.music.get_busy():  # if a song is currently playing
            mixer.music.stop()  # stop that song
        mixer.music.load(self.sound_path + data.data +".mp3")
        mixer.music.play(-1)  # play the new song indefinitely

if __name__ == '__main__':
    sp = Speaker()
    rospy.spin()

