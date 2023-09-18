#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from pygame import mixer
import rospkg
import os
import time


class Speaker:
    def __init__(self, namespace):
        
        rospy.init_node('speaker', anonymous=True)
        mixer.init()
        rospy.Subscriber("/"+ namespace+"/speaker", String, self.playSoundCallback)
        rospy.Subscriber("/"+ namespace+"/controller/state", String, self.playSoundCallback)
        rospy.Subscriber("/"+ namespace+"/wifiState", String, self.playSoundCallback)

        # rospack = rospkg.RosPack()
        # package_path = rospack.get_path('speaker')
        package_path = os.getcwd()
        # Define the sound file paths
        self.sound_path = package_path + "/sounds/"
        self.isPrioritySoundPlaying = False
        self.state="none"

    def playSoundCallback(self, data):
        rospy.loginfo(f"Received on : {data.data}")
        rospy.loginfo(f"[state] : {self.state}")


        while self.isPrioritySoundPlaying:
            time.sleep(0.1)


        if data.data == "moving":
            if self.state == "elevatorBoarding" or self.state == "elevatorGettingOff":
                return
            
            self.isPrioritySoundPlaying = False
            self.state = "moving"
            self.playEffectSound('moving.wav', -1)
            
        elif data.data == "emergencyStop":
            self.isPrioritySoundPlaying = False
            self.state = "emergencyStop"
            self.playEffectSound('emo.wav', -1)

        elif data.data == "robotError":
            self.playRobotErrorSound()


        elif data.data == "lowBattery":
            self.isPrioritySoundPlaying = True
            self.state = "lowBattery"
            
            self.playEffectSound('alarm.mp3', 1)
            
            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playVoiceSound('_voice_lowBattery.mp3', 1)
            

            
        elif data.data == "cannotMove":
            self.isPrioritySoundPlaying = True
            self.state = "cannotMove"

            self.playEffectSound('warning.mp3', 1)
            
            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playVoiceSound('_voice_cannotMove.mp3', 1)


        elif data.data == "obstacle":
            self.state ="obstacle"

            self.isPrioritySoundPlaying = False

            self.playEffectSound('obstacle.wav', 1)


        elif data.data == "deliveryCancel":
            self.state ="deliveryCancel"
            self.isPrioritySoundPlaying = True
            self.playEffectSound('cancel.mp3', 1)
            
            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playVoiceSound('_voice_deliveryCancel.mp3', 1)


        elif data.data == "deliveryPause":
            self.isPrioritySoundPlaying = False
            self.state = "deliveryPause"

            self.playEffectSound('pause.wav', -1)


        elif data.data == "deliveryReady":
            self.isPrioritySoundPlaying = True
            self.state = "deliveryReady"

            self.playVoiceSound('_voice_deliveryReady.mp3', 1)       

            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playEffectSound('request.mp3', 3)


        elif data.data == "elevatorBoarding":
            self.isPrioritySoundPlaying = True
            self.state = "elevatorBoarding"
            
            self.playVoiceSound('_voice_elevatorBoarding.mp3', 1)
            
            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playEffectSound('alarm.mp3', -1)


        elif data.data == "elevatorGettingOff":
            self.isPrioritySoundPlaying = True
            self.state = "elevatorGettingOff"

            self.playVoiceSound('_voice_elevatorGettingOff.mp3', 1)
            
            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playEffectSound('alarm.mp3', -1)


        elif data.data == "deliveryArrival":
            self.isPrioritySoundPlaying = True
            self.state = "deliveryArrival"

            self.playEffectSound('arrived.wav', 1)
            
            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playVoiceSound('_voice_deliveryArrival.mp3', 1)


        elif data.data == "deliveryPickupRequest":
            self.isPrioritySoundPlaying = True
            self.state = "deliveryPickupRequest"
            
            self.playVoiceSound('_voice_deliveryPickupRequest.mp3', 1)
            
            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playEffectSound('request.mp3', -1)


        elif data.data == "returnStart":
            self.isPrioritySoundPlaying = True
            self.state = "returnStart"

            self.playEffectSound('finish.wav', 1)
            
            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playVoiceSound('_voice_returnStart.mp3', 1)


        elif data.data =="soundOff":
            self.state = "soundOff"
            mixer.music.stop()

        elif data.data =="DISCONNECTED":
            if self.state == "DISCONNECTED" or self.state == "wifi_disconnection":
                return
            self.state ="DISCONNECTED"
            self.playRobotErrorSound()

        elif data.data =="wifi_disconnection":
            self.state ="wifi_disconnection"
            self.playRobotErrorSound()

        elif data.data =="CONNECTED":
            self.state ="CONNECTED"
            return

        else :
            rospy.logwarn(f" undefined meg received on : {data.data}")


    def playEffectSound(self, soundName, playCount):
        mixer.music.load(self.sound_path + soundName)
        mixer.music.play(playCount)

    def playVoiceSound(self, soundName, playCount):
        mixer.music.load(self.sound_path + soundName)
        mixer.music.play(playCount)
        
        while mixer.music.get_busy():
            time.sleep(0.1)
        self.isPrioritySoundPlaying = False
        

    def playRobotErrorSound(self):
        self.isPrioritySoundPlaying = True
        
        self.playEffectSound('alarm.mp3', 1)
        
        while mixer.music.get_busy():
            time.sleep(0.1)

        self.playVoiceSound('_voice_robotError.mp3', 1)

    def clean_filename(self, filename):
        # 확장자 제거
        if filename.endswith('.mp3'):
            filename = filename[:-4]
        elif filename.endswith('.wav'):
            filename = filename[:-4]

        # 앞의 _voice_ 제거
        if filename.startswith('_voice_'):
            filename = filename[7:]

        return filename        

import argparse
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--ns", type=str, required=True, help="Namespace for the robot")
    args = parser.parse_args()
    namespace = args.ns

    sp = Speaker(namespace)
    rospy.spin()

