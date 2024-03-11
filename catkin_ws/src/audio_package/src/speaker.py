from gtts import gTTS
import os

def convert_text_to_mp3(text, language='en'):
  myobj = gTTS(text=text, lang=language, slow=False)
  myobj.save("retro.mp3")
  os.system("mpg123 retro.mp3")
  print(text)
   

# Appel de la fonction main
if __name__ == "__main__":
    #convert_text_to_mp3(transcribed_text, language='en')
    convert_text_to_mp3('Very welcome, thank you !', language='en')