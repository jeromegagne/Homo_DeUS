import argparse
import os
import numpy as np
import speech_recognition as sr
import whisper
import torch
import speaker
from datetime import datetime, timedelta
from playsound import playsound
from time import sleep
from sys import platform
import threading
import queue
import requests
from gtts import gTTS

class TranscriberThread(threading.Thread):
    def __init__(self, args, queue):
        threading.Thread.__init__(self)
        self.args = args
        self.phrase_time = None
        self.data_queue = queue
        self.recorder = sr.Recognizer()
        self.recorder.energy_threshold = args.energy_threshold
        self.recorder.dynamic_energy_threshold = False
        self.source = self.get_microphone()
        self.audio_model = self.load_model()
        self.record_timeout = args.record_timeout
        self.phrase_timeout = args.phrase_timeout
        self.transcription = ['']


    def get_microphone(self):
        if 'linux' in platform:
            mic_name = self.args.default_microphone
            if not mic_name or mic_name == 'list':
                print("Available microphone devices are: ")
                for index, name in enumerate(sr.Microphone.list_microphone_names()):
                    print(f"Microphone with name \"{name}\" found")
                return
            else:
                for index, name in enumerate(sr.Microphone.list_microphone_names()):
                    if mic_name in name:
                        return sr.Microphone(sample_rate=16000, device_index=index)
        else:
            return sr.Microphone(sample_rate=16000)
    def load_model(self):
        model = self.args.model
        if self.args.model != "large" and not self.args.non_english:
            model = model + ".en"
        return whisper.load_model(model)

    def record_callback(self, _, audio:sr.AudioData) -> None:
        data = audio.get_raw_data()
        self.data_queue.put(data)


    
    def transcribe(self):
        with self.source:
            self.recorder.adjust_for_ambient_noise(self.source)
        self.recorder.listen_in_background(self.source, self.record_callback, phrase_time_limit=self.record_timeout)
        print("Model loaded.\n")
        while True:
            try:
                now = datetime.utcnow()
                if not self.data_queue.empty():
                    phrase_complete = False
                    if self.phrase_time and now - self.phrase_time > timedelta(seconds=self.phrase_timeout):
                        phrase_complete = True
                    self.phrase_time = now
                    # Convertir toutes les chaînes en bytes avant de les joindre
                    audio_data = b''.join(item.encode('utf-8') if isinstance(item, str) else item for item in self.data_queue.queue)
                    self.data_queue.queue.clear()
                    audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
                    result = self.audio_model.transcribe(audio_np, fp16=torch.cuda.is_available())
                    text = result['text'].strip()
                    if phrase_complete:
                        self.transcription.append(text)
                    else:
                        self.transcription[-1] = text
                    os.system('cls' if os.name=='nt' else 'clear')
                    for line in self.transcription:
                        if isinstance(line, bytes):
                            continue
                        self.data_queue.put(line)
                        
                    print('', end='', flush=True)
                    sleep(0.25)
            except KeyboardInterrupt:
                break

    def run(self):
        self.transcribe()

class ProcessorThread(threading.Thread):
    def __init__(self, queue):
        threading.Thread.__init__(self)
        self.queue = queue

    def play_and_delete(self, file_path):
        # Play the audio file
        playsound(file_path)

        # Delete the file after it has finished playing
        os.remove(file_path)

    def run(self):
        while True:
            try:
                text = self.queue.get()
                response = requests.post('http://localhost:5005/webhooks/rest/webhook', json={"sender": "Client", "message": text})
                print("Bot says, ")
                for i in response.json():
                    print(f"{i['text']}")
                    # Convertir le texte en parole
                    tts = gTTS(text=i['text'], lang='en')
                    # Enregistrer la parole dans un fichier
                    tts.save("response.mp3")
                    # Lire le fichier
                    self.play_and_delete("response.mp3")
            except queue.Empty:
                pass



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="tiny", help="Model to use",
                        choices=["tiny", "base", "small", "medium", "large"])
    parser.add_argument("--non_english", action='store_true',
                        help="Don't use the english model.")
    parser.add_argument("--energy_threshold", default=1000,
                        help="Energy level for mic to detect.", type=int)
    parser.add_argument("--record_timeout", default=2,
                        help="How real time the recording is in seconds.", type=float)
    parser.add_argument("--phrase_timeout", default=3,
                        help="How much empty space between recordings before we "
                             "consider it a new line in the transcription.", type=float)
    if 'linux' in platform:
        parser.add_argument("--default_microphone", default='pulse',
                            help="Default microphone name for SpeechRecognition. "
                                 "Run this with 'list' to view available Microphones.", type=str)
    args = parser.parse_args()

    transcript_queue = queue.Queue()
    transcriber_thread = TranscriberThread(args, transcript_queue)
    processor_thread = ProcessorThread(transcript_queue)
    transcriber_thread.start()
    processor_thread.start()