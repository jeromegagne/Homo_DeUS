import whisper
import os
import sounddevice as sd
from scipy.io.wavfile import write

import rospy

class ListenerNode:
    def __init__(self, model_name="tiny", fs=44100, seconds=5, audio_filename='output.wav'):
        self.model = whisper.load_model(model_name)
        self.fs = fs
        self.seconds = seconds
        self.audio_filename = audio_filename

    def record_audio(self):
        print("Enregistrement...")
        myrecording = sd.rec(int(self.seconds * self.fs), samplerate=self.fs, channels=2)
        sd.wait()  # Attendre la fin de l'enregistrement
        print("Enregistrement terminé")
        write(self.audio_filename, self.fs, myrecording)  # Sauvegarder en tant que fichier WAV

    def transcribe_audio(self):
        result = self.model.transcribe(self.audio_filename, verbose=True, language="en")
        print(result["text"])
        for i, seg in enumerate(result['segments']):
            print(i+1, "- ", seg['text'])
        return result["text"]

    def run(self):
        while not rospy.is_shutdown():
            self.record_audio()
            self.transcribe_audio()
        

    def main(self):
        rospy.init_node('listenerNode')
        listener = ListenerNode()
        listener.run()

if __name__ == "__main__":
    listener = ListenerNode()
    listener.main()