from gtts import gTTS
import os
import intake_messages as im
from io import BytesIO


folder_path = '/Users/raynahata/Desktop/Github/quori_ros/src/quori_exercises/speech_files/' 
if not os.path.exists(folder_path): 
    os.makedirs(folder_path) 

def text_to_speech(text):
    text=str(text)
    
    text_quote = text.replace("'", "")
    text_quote = text_quote.replace("]", "")

    # Initialize gTTS with the text to convert
    speech = gTTS(text_quote, lang='en')
   

    # Save the audio file to a temporary file
    speech_file = 'speech.mp3'
    speech.save(speech_file)

    # Play the audio file
    os.system('afplay ' + speech_file)


def main(): #tester main function 
    text=im.INTAKE_MESSAGES["Introduction"]["Greeting"]
    text_to_speech(text)



