#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Esto es necesario ya que los scripts de Python2 usan por defecto la codificación ASCII
# y por eso no "entiende" algunos símbolos, letras (ñ) o las tíldes que están presentes
# en el español pero no en el inglés.

"""
This module includes the definition to be able to check a figure features

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
        __figure (str): A string name of figure received
        __color (str): A string name of color received

    Methods:
        check(): Voice synthesis process manager function
        str_to_object(): Convert string values to object with range and curves
        detect_color(): Detect image color with a mask that has been created with figure features (range and curves)
        detect_shape(): Detect image shape with a mask that has been created with figure features (contours and curves)
    """

    def __init__(self, figure, color):
        """
        Initialize a Figure instance

        Args:
            figure (str): figure
            color (str): color
        """
        self.figure = figure
        self.color = color

    def check(self, cv_image):
        """ Check figure features of the image capture received 

        Args:
            cv_image (OpenCV): CV2 image

        Returns:
            state (Boolean): Returns the status of the comparison between the requested image and the received image capture 
        """
        figure_features = self.str_to_object()
        state = self.detect_color(cv_image, figure_features)
        return state

    def str_to_object(self):
        """ Convert string values to object with range and curves

        Returns:
            result ({curves, lower, upper}): Figure features of the image capture received
        """
        # variables y rangos
        result = dict()

        # rango de azul
        blue_lower = [94, 80, 2]
        blue_upper = [120, 255, 255]
        # rango de rojo
        red_lower = [0, 50, 50]
        red_upper = [15, 255, 255]
        # rango de verde
        green_lower = [25, 52, 72]
        green_upper = [102, 255, 255]

        if(self.color == "azul"):
            if(self.figure == "cuadrado"):
                result.update(
                    {"curves": "4", "lower": blue_lower, "upper": blue_upper})
            if(self.figure == "circulo"):
                result.update(
                    {"curves": "1", "lower": blue_lower, "upper": blue_upper})
            if(self.figure == "triangulo"):
                result.update(
                    {"curves": "3", "lower": blue_lower, "upper": blue_upper})
        if(self.color == "rojo"):
            if(self.figure == "cuadrado"):
                result.update(
                    {"curves": "4", "lower": red_lower, "upper": red_upper})
            if(self.figure == "circulo"):
                result.update(
                    {"curves": "1", "lower": red_lower, "upper": red_upper})
            if(self.figure == "triangulo"):
                result.update(
                    {"curves": "3", "lower": red_lower, "upper": red_upper})
        if(self.color == "verde"):
            if(self.figure == "cuadrado"):
                result.update(
                    {"curves": "4", "lower": green_lower, "upper": green_upper})
            if(self.figure == "circulo"):
                result.update(
                    {"curves": "1", "lower": green_lower, "upper": green_upper})
            if(self.figure == "triangulo"):
                result.update(
                    {"curves": "3", "lower": green_lower, "upper": green_upper})

        return result

    def detect_color(self, img, figure_features):
        """ Detect image color with a mask that has been created with figure features (range and curves)

        Args:
            img (OpenCV): CV2 image
            figure_features (Dict): Dictionary of figure features of the image capture received

        Returns:
             state (Boolean): If figure is valid or not
        """

        # low_value, up_value, curves
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        low_value = figure_features.get('lower')
        up_value = figure_features.get('upper')
        curves = int(figure_features.get('curves'))

        # Define mask
        color_lower = np.array(low_value, np.uint8)
        color_upper = np.array(up_value, np.uint8)
        color_mask = cv2.inRange(img_hsv, color_lower, color_upper)
        kernal = np.ones((5, 5), "uint8")

        # For x color
        color_mask = cv2.dilate(color_mask, kernal)
        res_color = cv2.bitwise_and(img, img, mask=color_mask)

        cv2.imshow("MASK", color_mask)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # Creating contour to track color
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
                is_shape = self.detect_shape(contours, curves)
                return is_shape

        return False

    def detect_shape(self, contours, curves):
        """ Detect image shape with a mask that has been created with figure features (contours and curves)

        Args:
            contours (Iterable): figure contours 
            curves (str): figure curves

        Returns:
            state (Boolean): If figure is valid or not
        """

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
