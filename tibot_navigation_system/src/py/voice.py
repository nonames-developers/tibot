# -*- coding: utf-8 -*-

# Esto es necesario ya que los scripts de Python2 usan por defecto la codificación ASCII
# y por eso no "entiende" algunos símbolos, letras (ñ) o las tíldes que están presentes
# en el español pero no en el inglés.

"""
This module includes the definition to be able to load a customize sound

Classes:
    Voice

Returns:
    Voice: A Voice instance
"""

from boto3 import Session
from botocore.exceptions import BotoCoreError, ClientError
from contextlib import closing
import os
import sys
import subprocess
from tempfile import gettempdir


class Voice:
    """
    Representation of a customize sound

    Attributes:
        __language (str): Examples: es-ES, en-US, en-IN ...
        __output_format (str): Possible values: json (voice marks) | mp3 | ogg_vorbis | pcm
        __sample_rate (str): Audio frequency: Recommended 22050 Hz
        __text_type (str): Specifies whether the input text is plain text or SSML
        __voice_id (str): Voice ID to use for synthesis: See DescribeVoices 
        __polly (Service): Service client instance
        __text (str): Text to synthesize

    Methods:
        load(): Voice synthesis process manager function 
        send_request(): Polly server request function 
        access_audio_from_response(): Access the audio stream from the response (binary stream in file)
        play_audio(): Play the audio using the platform's default player
    """

    def __init__(self, prof_name, language, output_format, sample_rate, text_type, voice_id):
        """
        Initialize a Voice instance

        Args:
            prof_name (str): The name of a profile to use
            language (str): Optional language code for request to synthesize speech
            output_format (str): The format in which the returned output will be encoded
            sample_rate (str): The specified audio frequency in Hz
            text_type (str): Specifies whether the input text is plain text or SSML 
            voice_id (str): Voice ID to use for synthesis
        """
        self.language = language
        self.output_format = output_format
        self.sample_rate = sample_rate
        self.text_type = text_type
        self.voice_id = voice_id

        session = Session(profile_name=prof_name)
        self.polly = session.client("polly")

    def load(self, text):
        """
        Voice synthesis process manager function 

        Args:
            text (str): Text to synthesize

        Returns:
            output (str): Binary stream in file 
        """
        self.text = text
        response = self.send_request()
        output = self.access_audio_from_response(response)
        return output

    def send_request(self):
        """
        Polly server request function

        Returns:
            response (AudioStream): Stream containing the synthesized speech 
        """
        try:
            # Request speech synthesis
            response = self.polly.synthesize_speech(
                LanguageCode=self.language,
                Text=self.text,
                OutputFormat=self.output_format,
                SampleRate=self.sample_rate,
                TextType=self.text_type,
                VoiceId=self.voice_id
            )
            return response
        except (BotoCoreError, ClientError) as error:
            print(error)
            sys.exit(-1)

    def access_audio_from_response(self, response):
        """
        Access the audio stream from the response

        Args:
            response (AudioStream): Stream containing the synthesized speech 

        Returns:
            output (str): Binary stream in file
        """
        if "AudioStream" in response:
            with closing(response["AudioStream"]) as stream:
                output = os.path.join(gettempdir(), "voice.ogg")
                try:
                    # Open a file for writing the output as a binary stream
                    with open(output, "wb") as file:
                        file.write(stream.read())
                        return output
                except IOError as error:
                    print(error)
                    sys.exit(-1)
        else:
            # The response didn't contain audio data, exit gracefully
            print("Could not stream audio")
            sys.exit(-1)

    def play_audio(self, output):
        """
        Play the audio using the platform's default player

        Args:
            output (str): Binary stream in file 
        """

        if sys.platform == "win32":
            os.startfile(output)
        else:
            opener = "open" if sys.platform == "darwin" else "xdg-open"
            print(output.format())
            subprocess.call([opener, output])
