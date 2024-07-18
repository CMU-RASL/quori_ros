import whisper
import glob
from pydub import AudioSegment as convert
import os
from whisper.utils import get_writer


import os
from pydub import AudioSegment

def convert_m4a_to_mp3():
    original_files_path = "/Users/raynahata/Desktop/Github/quori_ros_rayna/src/quori_exercises/audio_files/original_files"  
    converted_files_path = "/Users/raynahata/Desktop/Github/quori_ros_rayna/src/quori_exercises/audio_files/mp3_files"  
    # Loop through all files in the directory
    for filename in os.listdir(original_files_path):
        if filename.endswith(".m4a"):
            # Construct full file path
            input_file = os.path.join(original_files_path, filename)
            # Create the output file name by replacing the .m4a extension with .mp3
            output_file = os.path.join(converted_files_path, filename[:-4] + ".mp3")
            if not os.path.exists(output_file):

                # Load M4A file
                audio = AudioSegment.from_file(input_file, format="m4a")

                # Convert to MP3
                audio.export(output_file, format="mp3")
                print(f"Converted {filename} to MP3.")
            else:
                print(f"MP3 file already exists for {filename}, skipping conversion.")



def get_transcription(audio_file):
    output_folder="/Users/raynahata/Desktop/Github/quori_ros_rayna/src/quori_exercises/audio_files/transcription"
    # Load the Whisper model

    model = whisper.load_model("base")
    
    # Perform transcription
    result = model.transcribe(audio_file)
    
    txt_writer = get_writer("txt", output_folder)
    txt_writer(result, audio_file)

    
    return f"Transcription saved to {output_folder}"


if __name__ == "__main__":
   
    participant_number = 8
    audio_file_path = f"/Users/raynahata/Desktop/Github/quori_ros_rayna/src/quori_exercises/audio_files/mp3_files/Participant {participant_number}.mp3"

    convert_m4a_to_mp3()
    
    get_transcription(audio_file_path)



