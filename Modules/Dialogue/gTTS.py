
from gtts import gTTS
import os

mytext = 'Do you want a coffee?'
language = 'en'
myobj = gTTS(text=mytext, lang=language, slow=False)
myobj.save("retro.mp3")

os.system("mpg321 retro.mp3")