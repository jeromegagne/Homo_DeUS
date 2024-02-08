#!/usr/bin/env python

import rospy
from audio_common_msgs.msg import AudioData
from pydub import AudioSegment
from pydub.playback import play
import numpy as np
import io
import wave

def audio_callback(data):
    # Process audio data here
    wav_io = io.BytesIO(data.data)
    with wave.open(wav_io, 'rb') as wav_file:
        audio_data = np.frombuffer(wav_file.readframes(wav_file.getnframes()), dtype=np.int16)

    # Log that a message has been received
    rospy.loginfo("Received audio data with length: %s", len(audio_data))

    # Convert the audio data to an AudioSegment
    audio_segment = AudioSegment(
        audio_data.tobytes(), 
        frame_rate=44100,
        sample_width=2,
        channels=1
    )

    # Play the audio segment
    play(audio_segment)

def audio_subscriber():
    rospy.init_node('audio_subscriber', anonymous=False)
    rospy.Subscriber("/audio", AudioData, audio_callback)
    rospy.loginfo("Audio subscriber node is running. Press Ctrl+C to exit.")
    rospy.spin()

if __name__ == '__main__':
    try:
        audio_subscriber()
    except rospy.ROSInterruptException:
        pass