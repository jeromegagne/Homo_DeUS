import whisper
import os

# Record audio from microphone
import sounddevice as sd
from scipy.io.wavfile import write
import wavio as wv

model = whisper.load_model("base")



fs = 44100  # Sample rate
seconds = 3  # Duration of recording
print("Recording...")
myrecording = sd.rec(int(seconds * fs), samplerate=fs, channels=2)
sd.wait()  # Wait until recording is finished
print("Finished recording")
write('output.wav', fs, myrecording)  # Save as WAV file


result = model.transcribe("output.wav", verbose = True)
print(result["text"])
result['segments']

for i, seg in enumerate(result['segments']):
  print(i+1, "- ", seg['text'])