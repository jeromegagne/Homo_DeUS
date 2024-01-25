import whisper
import os

from gtts import gTTS

# Record audio from microphone
import sounddevice as sd
from scipy.io.wavfile import write

#NLP
import nltk
#from nltk.tokenize import word_tokenize
#from nltk.corpus import stopwords
#from nltk.stem import WordNetLemmatizer

model = whisper.load_model("base")

def record_audio(fs=44100, seconds=5, audioFilename='output.wav'):
  print("Enregistrement...")
  myrecording = sd.rec(int(seconds * fs), samplerate=fs, channels=2)
  sd.wait()  # Attendre la fin de l'enregistrement
  print("Enregistrement terminé")
  write(audioFilename, fs, myrecording)  # Sauvegarder en tant que fichier WAV

def transcribe_audio(model, filename="output.wav"):
  result = model.transcribe(filename, verbose = True)
  print(result["text"])
  for i, seg in enumerate(result['segments']):
    print(i+1, "- ", seg['text'])
  return result["text"]

# Étape de prétraitement
def preprocess_text(text):
  tokens = nltk.word_tokenize(text)
  tokens = [word.lower() for word in tokens if word.isalpha()]
  stop_words = set(nltk.stopwords.words('english'))
  tokens = [word for word in tokens if word not in stop_words]
  lemmatizer = nltk.WordNetLemmatizer()
  tokens = [lemmatizer.lemmatize(word) for word in tokens]
  return tokens

def convert_text_to_mp3(text, language='en'):
  myobj = gTTS(text=text, lang=language, slow=False)
  myobj.save("retro.mp3")
  os.system("start retro.mp3")

def main():
  # Enregistrement de l'audio
  audioFilename = 'output.wav'
  time = 5
  frequency = 44100
  record_audio(fs=frequency, seconds=time, audioFilename=audioFilename)
  # Transcription de l'audio
  transcribed_text = transcribe_audio(model, filename=audioFilename)
  preprocessed_text = preprocess_text(transcribed_text)
  print(preprocessed_text)
  # Conversion du texte transcrit en mp3
  #convert_text_to_mp3(transcribed_text, language='en')

# Appel de la fonction main
if __name__ == "__main__":
  main()