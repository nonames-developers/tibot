#! /usr/bin/env python

"""
This module includes the definition to be able to activate a service for the robot voices 

Classes:
    PlayVoice

Exceptions:
    ROSInterruptException: Exception for operations that interrupted, e.g. due to shutdown.

Returns:
    PlayVoice: A voice service instance

Yields:
    []
"""

import rospy
from voice import Voice
from sound_play.libsoundplay import SoundClient
from tibot_navigation_system.srv import VoiceMsg, VoiceMsgRequest, VoiceMsgResponse


class PlayVoice():
    """
    Representation of Tibot voice

    Attributes:
        service_server (Rospy.Service): Service to listen requests

    Methods:
        srv_callback(): Callback to play received phrase 
    """

    def __init__(self):
        rospy.init_node('play_voice_service', anonymous=True)
        self.service_server = rospy.Service(
            'play_voice_robot_service', VoiceMsg, self.srv_callback)
        rospy.loginfo("Service /play_voice_robot_service ready!")
        rospy.spin()  # Keep the service open

    def srv_callback(self, request):
        """ Function that receives a phrase to play it with voice

        Args:
            request (VoiceMsgRequest): A message that contains a phrase (text) to play

        Returns:
            response (VoiceMsgResponse): Success or not
        """

        # We obtain the data of the received message
        phrase_to_play = str(request.phrase)

        # Voice
        v = Voice("default", "es-ES", "ogg_vorbis", "22050", "text", "Miguel")
        output_path = v.load(phrase_to_play)

        # Client to publish a sound to the sound_play topic
        sound_client = SoundClient(blocking=True)

        # Ensure publisher connection is successful.
        rospy.sleep(0.5)

        # Method 1: Play Wave file directly from Client
        sound_client.playWave(output_path, volume=1.0)

        # Result of the execution from the service
        response = VoiceMsgResponse()
        response.success = True
        return response


def main():
    """ Main function to init PlayVoice

    Exceptions:
        ROSInterruptException: Exception for operations that interrupted, e.g. due to shutdown.
    """
    try:
        PlayVoice()
    except rospy.ROSInterruptException:
        rospy.loginfo("PlayVoice service has been finished.")

####################################################################################################


# Init
main()
