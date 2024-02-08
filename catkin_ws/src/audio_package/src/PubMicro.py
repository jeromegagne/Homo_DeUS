#!/usr/bin/env python

import rospy
from audio_common_msgs.msg import AudioData
import sounddevice as sd
import threading
import numpy as np
from scipy.io.wavfile import write
import io
from pydub import AudioSegment
from pydub.playback import play


class AudioPublisher:
    def __init__(self):
        self.audio_pub = rospy.Publisher('/audio', AudioData, queue_size=10)
        self.fs = 44100  # Sampling frequency in Hz
        self.recording = False  # Whether we are currently recording
        self.audio_data = np.array([])  # The audio data that has been recorded
        self.stream = None

    def start_recording(self):
        self.recording = True
        rospy.loginfo("Recording audio...")
        self.audio_data = np.array([])
        self.stream = sd.InputStream(callback=self.audio_callback)
        self.stream.start()

    def stop_recording(self):
        self.recording = False
        if self.stream is not None:
            self.stream.stop()
            self.stream.close()
        rospy.loginfo("Audio recorded")
    
        if self.audio_data.size > 0:
            # Convert audio data to int16
            audio_data_int16 = (self.audio_data * 32767).astype('int16')

            # Write audio data to a WAV file in memory
            wav_io = io.BytesIO()
            write(wav_io, self.fs, audio_data_int16)

            # Read the WAV file and send it
            wav_data = wav_io.getvalue()

            # Create an AudioData message and publish it
            audio_msg = AudioData(data=wav_data)
            self.audio_pub.publish(audio_msg)
    
            rospy.loginfo("Audio published")
    
    def audio_callback(self, indata, frames, time, status):
        if status:
            print("Error in audio input:", status)
            return

        if self.recording:
            self.audio_data = np.append(self.audio_data, indata.copy())

    def user_input_thread(self):
        while True:
            command = input("Enter 'start' to start recording, 'stop' to stop recording: ")
            if command == 'start':
                self.start_recording()
            elif command == 'stop':
                self.stop_recording()

    def run(self):
        rospy.init_node('audio_publisher', anonymous=False)
        rate = rospy.Rate(10)  # Set the publishing rate (e.g., 10 Hz)

        # Start a new thread for reading user input
        threading.Thread(target=self.user_input_thread, daemon=True).start()

        rospy.loginfo("Audio publisher node is running. Press Ctrl+C to exit.")
        rospy.spin()

if __name__ == '__main__':
    try:
        AudioPublisher().run()
    except rospy.ROSInterruptException:
        pass
