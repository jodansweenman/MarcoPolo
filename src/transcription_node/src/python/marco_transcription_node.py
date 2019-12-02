#!/usr/bin/python
"""
    marcotranscription_node.py
    Robert Holt
    This module implements the streaming transcription of live audio via
    Google Cloud services.

    Currently only runs for 5 mins max.  Next version will include unlimited transcription
"""

from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types

import rospy
from transcription_node.msg import TranscriptionResult
from transcribe_streaming_mic import MicrophoneStream
import pyaudio
from six.moves import queue

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms

def listen_publish_loop(responses, publisher):
    """Iterates through server responses and prints them.

    The responses passed is a generator that will block until a response
    is provided by the server.

    Each response may contain multiple results, and each result may contain
    multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
    print only the transcription for the top alternative of the top result.

    In this case, responses are provided for interim results as well. If the
    response is an interim one, print a line feed at the end of it, to allow
    the next result to overwrite it, until the response is a final one. For the
    final one, print a newline to preserve the finalized transcription.
    """

    for response in responses:
        if rospy.is_shutdown():
            rospy.loginfo('Shutting Down...')
            break

        if not response.results:
            # Then there was no result in this response
            continue

        # The `results` list is consecutive. For streaming, we only care about
        # the first result being considered, since once it `is_final`, it
        # moves on to considering the next utterance.
        result = response.results[0]
        if not result.alternatives:
            continue

        if result.is_final:
            # Display the transcription of the top alternative.
            transcript = result.alternatives[0].transcript
            confidence = result.alternatives[0].confidence
            rospy.loginfo("Final: '%s' w/ confidence %f" % (transcript, confidence))
            # Here is where it needs to be published
            transmsg = TranscriptionResult(transcribed_text=transcript,
                                           confidence=confidence)

            publisher.publish(transmsg)
        else:
            transcript = result.alternatives[0].transcript
            stability = result.stability
            rospy.loginfo("Interim: '%s' w/ stability %f" % (transcript, stability))

    return

def main():
    rospy.init_node(name='transcription_node')

    rospy.loginfo('Registering as publisher for /mpstate/transcription')
    trans_pub = rospy.Publisher('mpstate/transcription', TranscriptionResult, queue_size=10, latch=True)

    # Some short snippets which are likely to be told to Marco.
    marco_phrases = ['hey marco', 'where is the', 'go get the']

    language_code = 'en-US'  # a BCP-47 language tag

    client = speech.SpeechClient()
    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=language_code,
        phrases=marco_phrases)

    streaming_config = types.StreamingRecognitionConfig(
        config=config,
        interim_results=True)

    with MicrophoneStream(RATE, CHUNK) as stream:
        audio_generator = stream.generator()
        requests = (types.StreamingRecognizeRequest(audio_content=content)
                    for content in audio_generator)

        responses = client.streaming_recognize(streaming_config, requests)

        # Now, put the transcription responses to use.
        listen_publish_loop(responses, trans_pub)

    rospy.spin()

    return

if __name__ == "__main__":
    main()
