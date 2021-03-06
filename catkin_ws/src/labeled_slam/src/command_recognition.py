#!/usr/bin/env python

## @labeled_slam command_recognition.py
## Speech recognition and command node
# Requires connection to the internet

# Import system calls
import os

# Python libraries
import time
import speech_recognition as sr # Speech Recognition library

# ROS libraries
import roslib
import rospy

# ROS messages
from labeled_slam.msg import Command # Custome message containing two strings variables and a header

## Mother class
class Main_Speech_Controller:

        ## Initialize global variables like topics and recognizer
        def __init__(self):

            # Topic where we will publish our costume message
            self.mytopic= rospy.Publisher('text_command', Command, queue_size=10)

            # Structure for the speech recognition library result
            self.speech= {"Transcription":None, "Success":True, "Error":None}

            # Feedback from user
            self.confirmation= 0

            # State of the speech recognition operations
            self.status= 0

            # Initialize a message type speechcommand
            self.speechcommand= Command()
            self.speechcommand.command= "" # Tried to used None insted of "" but mytopic.pub returned error
            self.speechcommand.argument= ""

        ## Get the audio signal from the microphone as a string
        def recognize_speech(self, speech):

            # Declare a type of recognizer an microphone for the speech recognition library
            speechrecognizer= sr.Recognizer()
            microphone= sr.Microphone()


            with microphone as source:
                # Identifies the backgroundnoise to recognize better the speech
                speechrecognizer.adjust_for_ambient_noise(source)
                # Listen
                print("***** Listening...")
                audio= speechrecognizer.listen(source)

            # Troubleshoot
            try:
                speech["Transcription"]= speechrecognizer.recognize_google(audio)
            except sr.RequestError:
                speech["Success"]= False
                speech["Error"]= "API unavailable"
            except sr.UnknownValueError:
                speech["Error"]= "Unable to recognize the past speech"


            return speech

        ## From the audio signal received, publish the desired command
        def recognize_command(self, speech, speechcommand, mytopic):

            if speech["Transcription"]== "drive":

                # Update the message variables
                speechcommand.command= speech["Transcription"]
                speechcommand.argument= ""

                # Publish the speechcommand message to /speechcommandtopic
                mytopic.publish(speechcommand)

                print("***** Driving mode")
                return 1

            elif speech["Transcription"]== "go to":

                # Update the first message variable
                speechcommand.command= speech["Transcription"]

                # Listen to the second argument of the command
                print("***** Where should we go?")
                speech=self.recognize_speech(self.speech)

                # Ask if user agrees with the recognized word
                confirmation=self.recognize_user(self.speech, self.confirmation)

                # If yes
                if confirmation==1:

                    # ***** MISSING *****
                    # Is it a valid location?
                    # If yes

                    # Update the second message variable
                    speechcommand.argument= speech["Transcription"]

                    # Publish the speechcommand message to /speechcommandtopic
                    mytopic.publish(speechcommand)

                    print("***** Going autonomously to: " + speech["Transcription"])
                    return 1

                    # ***** MISSING *****
                    # Otherwise, start all over again
                    # else:
                    #   print("Sure, but first tell where is it at")
                    #   return 0

                # Otherwise, start all over again
                else:
                    print("***** Oops! Sorry about that")
                    #time.sleep(3)
                    return 0

            elif speech["Transcription"]=="label":

                # Update the first message variable
                speechcommand.command= speech["Transcription"]

                # Listen to the second argument of the command
                print("***** How should we call it? Be creative")
                speech=self.recognize_speech(self.speech)

                # Ask if the user agrees with the recognized word
                confirmation=self.recognize_user(self.speech, self.confirmation)

                # If yes
                if confirmation==1:

                    # Update the second message variable
                    speechcommand.argument= speech["Transcription"]

                    # Publish the speechcommand message to /speechcommandtopic
                    mytopic.publish(speechcommand)

                    print("***** " + speech["Transcription"] + "saved successfully")
                    return 1

                # Otherwise, start all over again
                else:
                    print("***** Oops! Sorry about that")
                    #time.sleep(3)
                    return 0

            # If command not valid
            else:

                # Update message variables
                speechcommand.command= ""
                speechcommand.argument= ""

                # Start all over again
                print("***** I'm afraid I cannot do that")
                return 0

        ## Ask user for feedback of the recognized speech
        def recognize_user(self, speech, confirmation):

            # If the speech is free of errors
            if speech["Success"]==1:

                # Ask if the user agrees with the recognized speech
                print("***** Did you say: " + speech["Transcription"])
                print("----------")

                # Wait for keyboard input, 1=YES 0=NO
                confirmation=int(input("Type 1 if yes or 0 otherwise: "))
                return confirmation

            # If the recognized speech has an error
            else:

                # Start all over again
                print("***** An error ocurred", speech["Error"])
                confirmation= 0
                return confirmation

        ## The 'main' function of the class, it calls constantly the other
        def start_recognition(self):

            while not rospy.is_shutdown():

                # Clear command window
                # os.system('clear')

                # Ask if the user wants to perform a new voice command
                print("****************************************")
                print("***** Do you want to perform a voice command now? Only 'drive', 'go to' and 'label' are valid (for now)")
                print("----------")

                # Wait for keyboard input, 1=YES 0=NO
                aux=int(input("Type 1 if yes or 0 otherwise: "))

                # If yes
                if aux==1:

                    # Publish a message saying that we are Listening
                    self.speechcommand.command= "listen"
                    self.speechcommand.argument= ""
                    self.mytopic.publish(self.speechcommand)

                    # Recognize speech
                    self.speech= self.recognize_speech(self.speech)

                    # Ask for feedback
                    self.confirmation= self.recognize_user(self.speech, self.confirmation)

                    # If it is good
                    if self.confirmation==1:

                        # Get the right command and publish it
                        self.status= self.recognize_command(self.speech, self.speechcommand, self.mytopic)

                        # Show status of the operation
                        if self.status==1:
                            print("***** command sent")
                        else:
                            print("***** command NOT sent")
                        #time.sleep(3)

                    # Otherwise, start all over again
                    else:
                        print("***** Oops! sorry about that")
                        #time.sleep(3)

                # Otherwise
                else:

                    # Sleep for 10 seconds
                    print("***** It's okay, I'm going to sleep 10 seconds. If you want to exit, press CTRL+C")
                    print("****************************************")
                    time.sleep(5) # Doing this mostly for having an exit door for the program / Might change it later

## Main, initializes the class and starts the node
def main():

    # Initialize and cleanup the ROS node
    rospy.init_node('Main_Speech_Controller', anonymous=True)

    # Declare a new instance of the class and run its starting function
    MSC= Main_Speech_Controller()
    MSC.start_recognition()

    # Listen to CTRL+C interruptions when is not waiting for inputs
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting down the Main Speech Controller node")
        pass

if __name__ == '__main__':
    main()


# ----------
## TO-DO list:
# 1. How to check if the label of Go mode is correct?
# 2. Fill with error exceptions
# 3. Might do a GUI if we have time
