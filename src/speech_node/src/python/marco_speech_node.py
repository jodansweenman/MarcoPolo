#!/usr/bin/python
"""
    marco_speech_node.py
    This module implements the speaking and synthesis of responses to questions from a user
"""
import boto3
import pygame
import rospy
from speech_node.msg import SynthRq, PlayRq
import os

class MarcoSpeaker:
    """
    This class implements functionality for synthesizing and
    playing back data when requested via ROS
    """

    def __init__(self):
        rospy.loginfo('Initializing pygame for playback')
        pygame.init()
        pygame.mixer.init()

        rospy.loginfo('Initializing Polly client')
        self.polly_client = boto3.Session().client('polly')
        
        rospy.loginfo("Subscribing to mpstate/synth_rq")
        rospy.Subscriber('mpstate/synth_rq', SynthRq, callback=self.handle_synth_rq)

        rospy.loginfo("Subscribing to mpstate/play_rq")
        rospy.Subscriber('mpstate/play_rq', PlayRq, callback=self.handle_play_rq)

        self.basePath = rospy.get_param('/marcopolo/mp3_path', os.path.dirname(os.path.abspath(__file__)))

        self.synth_jobs = {}

        return

    def _get_synth_result(self, text, client):
        rospy.loginfo("Getting result for '%s'" % text)

        response = client.synthesize_speech(VoiceId='Brian',
                                            OutputFormat='mp3',
                                            Text=text)
        rospy.loginfo("Synthesis complete")

        return response

    def _write_audio_file(self, polly_response, file_name='speech.mp3'):
        rospy.loginfo("Writing audio file to '%s'" % file_name)
        f = open(file_name, 'wb')
        f.write(polly_response['AudioStream'].read())
        f.close()
        return

    def handle_play_rq(self, data):
        synth_name = data.synth_name

        if synth_name in self.synth_jobs.keys():
            self._play_audio_file(file_path=self.synth_jobs[synth_name])
        else:
            self._play_audio_file(file_path='ball_not_found.mp3')

    def _play_audio_file(self, file_path):
        rospy.loginfo("Loading %s for playback" % file_path)
        pygame.mixer.music.load(file_path)
        pygame.time.Clock().tick(1)

        pygame.mixer.music.play()
        rospy.loginfo('Beginning Playback ...')

        while pygame.mixer.music.get_busy():
            rospy.loginfo('in loop')
            pygame.time.Clock().tick(10)
        rospy.loginfo('Post loop')
        return

    def handle_synth_rq(self, data):
        synth_name = data.synth_name
        synth_filename = synth_name + ".mp3"
        synth_text = data.synth_text

        rospy.loginfo("Synthesizing text for '%s'" % synth_text)
        response = self._get_synth_result(synth_text, self.polly_client)

        synth_fullname = os.path.join(self.basePath, synth_filename)

        rospy.loginfo("Writing file: '%s'" % synth_fullname)
        self._write_audio_file(response, file_name=synth_fullname)

        self.synth_jobs[synth_name] = synth_fullname

        return

def main():

    rospy.init_node(name='marco_speech_node')

    ms = MarcoSpeaker()

    rospy.spin()

    return

if __name__ == "__main__":
    main()
