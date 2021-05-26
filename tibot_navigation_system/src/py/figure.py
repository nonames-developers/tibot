#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Esto es necesario ya que los scripts de Python2 usan por defecto la codificación ASCII
# y por eso no "entiende" algunos símbolos, letras (ñ) o las tíldes que están presentes
# en el español pero no en el inglés.

"""
This module includes the definition to be able to check a figure

Classes:
    Figure

Returns:
    Figure: A Figure instance
"""

from typing import Counter
import rospy
import cv2
import numpy as np


class Figure:
    """
    Representation of a figure

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

    def __init__(self, figure, color):
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
        self.figure = figure
        self.color = color

    def check(self, cv_image):
        figure_features = self.str_to_object()
        state = self.detect_color(cv_image, figure_features)
        return state

    def str_to_object(self):
        """ Convert strings to our Obj
        Args:
            figure (str): figure
            color (str): color
        Returns:
            result {(CURVES, LOWER, UPPER)}: Obj
        """
        # variables y rangos
        result = dict()

        # rango de azul
        blue_lower = [94, 80, 2]
        blue_upper = [240, 255, 180]
        # rango de rojo
        red_lower = [0, 50, 50]
        red_upper = [15, 255, 255]
        # rango de verde
        green_lower = [25, 52, 72]
        green_upper = [102, 255, 255]

        if(self.color == "azul"):
            if(self.figure == "cuadrado"):
                result.update({"curves": "4", "lower": blue_lower, "upper": blue_upper})
            if(self.figure == "circulo"):
                result.update({"curves": "1", "lower": blue_lower, "upper": blue_upper})
            if(self.figure == "triangulo"):
                result.update({"curves": "3", "lower": blue_lower, "upper": blue_upper})
        if(self.color == "rojo"):
            if(self.figure == "cuadrado"):
                result.update({"curves": "4", "lower": red_lower, "upper": red_upper})
            if(self.figure == "circulo"):
                result.update({"curves": "1", "lower": red_lower, "upper": red_upper})
            if(self.figure == "triangulo"):
                result.update({"curves": "3", "lower": red_lower, "upper": red_upper})
        if(self.color == "verde"):
            if(self.figure == "cuadrado"):
                result.update({"curves": "4", "lower": green_lower, "upper": green_upper})
            if(self.figure == "circulo"):
                result.update({"curves": "1", "lower": green_lower, "upper": green_upper})
            if(self.figure == "triangulo"):
                result.update({"curves": "3", "lower": green_lower, "upper": green_upper})

        return result

    def detect_color(self, img, figure_features):

        # low_value, up_value, curves
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        low_value = figure_features.get('lower')
        up_value = figure_features.get('upper')
        curves = figure_features.get('curves')

        # define mask
        color_lower = np.array(low_value, np.uint8)
        color_upper = np.array(up_value, np.uint8)
        color_mask = cv2.inRange(img_hsv, color_lower, color_upper)
        kernal = np.ones((5, 5), "uint8")

        # For red color
        color_mask = cv2.dilate(color_mask, kernal)
        res_color = cv2.bitwise_and(img, img,
                                    mask=color_mask)
        # Creating contour to track red color
        _, contours, _ = cv2.findContours(color_mask,
                                          cv2.RETR_TREE,
                                          cv2.CHAIN_APPROX_SIMPLE)

        for _, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 100):

                x, y, w, h = cv2.boundingRect(contour)
                img = cv2.rectangle(img, (x, y), 
                                        (x + w, y + h), 
                                        (0, 0, 255), 2)
                cv2.putText(img, "Red Colour", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 0, 255))

                shape = self.detect_shape(contours, curves)
                return shape
            return False

    def detect_shape(self, contours, curves):

        for contorno in contours:
            poligonoAproximado = cv2.approxPolyDP(
                contorno, 0.01 * cv2.arcLength(contorno, True), True)

            numeroCurvas = len(poligonoAproximado)

            if numeroCurvas == 3 and numeroCurvas == curves:
                return True
            elif numeroCurvas == 4 and numeroCurvas == curves:
                print("He encontrado el cuadrado")
                return True
            elif 1 == curves:
                return True

            return False
