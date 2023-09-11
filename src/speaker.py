#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from pygame import mixer
import rospkg
import os



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
        


    def playSoundCallback(self, data):
        # Check and stop if any music is currently playing
        for i in range(mixer.get_num_channels()):
            channel = mixer.Channel(i)
            if channel.get_busy():
                channel.stop()
        
        rospy.loginfo(f"Received on : {data.data}")

        effect_channel = mixer.Channel(0)
        voice_channel = mixer.Channel(1)

        if data.data == "moving":
            effect_sound = mixer.Sound(self.sound_path + 'moving.wav')
            effect_channel.play(effect_sound, loops=-1)
            
        elif data.data == "emergencyStop":
            effect_sound = mixer.Sound(self.sound_path + 'emo.wav')
            effect_channel.play(effect_sound, loops=-1)  

        elif data.data == "robotError":
            effect_sound = mixer.Sound(self.sound_path + 'alarm.mp3')
            voice_sound = mixer.Sound(self.sound_path + '_voice_robotError.mp3')  

            effect_channel.play(effect_sound)  
            voice_channel.play(voice_sound)  

        elif data.data == "lowBattery":
            effect_sound = mixer.Sound(self.sound_path + 'alarm.mp3')
            voice_sound = mixer.Sound(self.sound_path + '_voice_lowBattery.mp3')  

            effect_channel.play(effect_sound)  
            voice_channel.play(voice_sound)  

        elif data.data == "cannotMove":
            effect_sound = mixer.Sound(self.sound_path + 'warning.mp3')
            voice_sound = mixer.Sound(self.sound_path + '_voice_cannotMove.mp3')  

            effect_channel.play(effect_sound)  
            voice_channel.play(voice_sound)  

        elif data.data == "obstacle":
            effect_sound = mixer.Sound(self.sound_path + 'obstacle.wav')
            effect_channel.play(effect_sound)  

        elif data.data == "deliveryCancel":
            effect_sound = mixer.Sound(self.sound_path + 'cancel.mp3')
            voice_sound = mixer.Sound(self.sound_path + '_voice_deliveryCancel.mp3')  

            effect_channel.play(effect_sound)  
            voice_channel.play(voice_sound)  

        elif data.data == "deliveryPause":
            effect_sound = mixer.Sound(self.sound_path + 'pause.wav')
            effect_channel.play(effect_sound, loops=-1)  

        elif data.data == "deliveryReady":
            effect_sound = mixer.Sound(self.sound_path + 'request.mp3')
            voice_sound = mixer.Sound(self.sound_path + '_voice_deliveryReady.mp3')  

            effect_channel.play(effect_sound, loops=2)  
            voice_channel.play(voice_sound)  

        elif data.data == "elevatorBoarding":
            effect_sound = mixer.Sound(self.sound_path + 'alarm.mp3')
            voice_sound = mixer.Sound(self.sound_path + '_voice_elevatorBoarding.mp3')  

            effect_channel.play(effect_sound, loops=-1)  
            voice_channel.play(voice_sound)  

        elif data.data == "elevatorGettingOff":
            effect_sound = mixer.Sound(self.sound_path + 'alarm.mp3')
            voice_sound = mixer.Sound(self.sound_path + '_voice_elevatorGettingOff.mp3')  

            effect_channel.play(effect_sound, loops=-1)  
            voice_channel.play(voice_sound)  

        elif data.data == "deliveryArrival":
            effect_sound = mixer.Sound(self.sound_path + 'arrived.wav')
            voice_sound = mixer.Sound(self.sound_path + '_voice_deliveryArrival.mp3')  

            effect_channel.play(effect_sound)  
            voice_channel.play(voice_sound)  

        elif data.data == "deliveryPickupRequest":
            effect_sound = mixer.Sound(self.sound_path + 'request.mp3')
            voice_sound = mixer.Sound(self.sound_path + '_voice_deliveryPickupRequest.mp3')  

            effect_channel.play(effect_sound, loops=-1)  
            voice_channel.play(voice_sound)  

        elif data.data == "returnStart":
            effect_sound = mixer.Sound(self.sound_path + 'finish.wav')
            voice_sound = mixer.Sound(self.sound_path + '_voice_returnStart.mp3')  

            effect_channel.play(effect_sound)  
            voice_channel.play(voice_sound)  

        else :
            rospy.logwarn(f" undefined meg received on : {data.data}")





if __name__ == '__main__':
    sp = Speaker()
    rospy.spin()

