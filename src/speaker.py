#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from pygame import mixer
import rospkg
import os
import time


class Speaker:
    def __init__(self):
        rospy.init_node('speaker', anonymous=True)
        mixer.init()
        rospy.Subscriber("speaker", String, self.playSoundCallback)

        
        # rospack = rospkg.RosPack()
        # package_path = rospack.get_path('speaker')
        package_path = os.getcwd()
        # Define the sound file paths
        self.sound_path = package_path + "/sounds/"
        self.isPrioritySoundPlaying = False


    def playSoundCallback(self, data):
        rospy.loginfo(f"Received on : {data.data}")

        while self.isPrioritySoundPlaying:
            time.sleep(0.1)


        if data.data == "moving":
            self.isPrioritySoundPlaying = False
            self.playEffectSound('moving.wav', -1)
            
        elif data.data == "emergencyStop":
            self.isPrioritySoundPlaying = False
            self.playEffectSound('emo.wav', -1)

        elif data.data == "robotError":
            self.isPrioritySoundPlaying = True
            self.playEffectSound('alarm.mp3', 1)
            
            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playVoiceSound('_voice_robotError.mp3', 1)

        elif data.data == "lowBattery":
            self.isPrioritySoundPlaying = True
            self.playEffectSound('alarm.mp3', 1)
            
            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playVoiceSound('_voice_lowBattery.mp3', 1)
            
        elif data.data == "cannotMove":
            self.isPrioritySoundPlaying = True
            self.playEffectSound('warning.mp3', 1)
            
            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playVoiceSound('_voice_cannotMove.mp3', 1)

        elif data.data == "obstacle":
            self.isPrioritySoundPlaying = False
            self.playEffectSound('obstacle.wav', 1)

        elif data.data == "deliveryCancel":
            self.isPrioritySoundPlaying = True
            self.playEffectSound('cancel.mp3', 1)
            
            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playVoiceSound('_voice_deliveryCancel.mp3', 1)

        elif data.data == "deliveryPause":
            self.isPrioritySoundPlaying = False
            self.playEffectSound('pause.wav', -1)

        elif data.data == "deliveryReady":
            self.isPrioritySoundPlaying = True
            self.playVoiceSound('_voice_deliveryReady.mp3', 1)       

            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playEffectSound('request.mp3', 3)

        elif data.data == "elevatorBoarding":
            self.isPrioritySoundPlaying = True
            self.playVoiceSound('_voice_elevatorBoarding.mp3', 1)
            
            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playEffectSound('alarm.mp3', -1)

        elif data.data == "elevatorGettingOff":
            self.isPrioritySoundPlaying = True
            self.playVoiceSound('_voice_elevatorGettingOff.mp3', 1)
            
            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playEffectSound('alarm.mp3', -1)

        elif data.data == "deliveryArrival":
            self.isPrioritySoundPlaying = True
            self.playEffectSound('arrived.wav', 1)
            
            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playVoiceSound('_voice_deliveryArrival.mp3', 1)

        elif data.data == "deliveryPickupRequest":
            self.isPrioritySoundPlaying = True
            self.playVoiceSound('_voice_deliveryPickupRequest.mp3', 1)
            
            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playEffectSound('request.mp3', -1)

        elif data.data == "returnStart":
            self.isPrioritySoundPlaying = True
            self.playEffectSound('finish.wav', 1)
            
            while mixer.music.get_busy():
                time.sleep(0.1)

            self.playVoiceSound('_voice_returnStart.mp3', 1)

        elif data.data =="soundOff":
            mixer.music.stop()

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



if __name__ == '__main__':
    sp = Speaker()
    rospy.spin()

