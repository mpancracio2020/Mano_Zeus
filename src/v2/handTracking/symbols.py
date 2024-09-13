import cv2
import numpy as np

# SYMBOLS NEADED
CHANGE_MODE_1 = [[1, 1, 0, 0, 0, 1, 1], [1, 1, 0, 0, 0, 0, 1]]
CHANGE_MODE_2 = [[1, 1, 0, 0, 0, 0, 1], [1, 1, 0, 0, 0, 1, 1]]
SYMBOLS_MODE = 0
MOVE_MODE = 1
FACE_MODE = 2
MANUAL_MODE = 3

PEACE = [0, 1, 1, 0, 0, 1, 1]
METAL = [0, 1, 0, 0, 1, 1, 1]
FUCK = [0, 0, 1, 0, 0, 0, 1]
GOOD = [1, 0, 0, 0, 0, 0, 1]
INVITA = [1, 0, 0, 0, 1, 0, 1]
TEMPLARIOS = [0, 1, 0, 0, 1, 0, 1]

def symbol(status, hand, traces):
    if status == PEACE:
        if traces:
            print("PEACE")
        if hand == 1:
            image = cv2.imread('symbols/peace1.jpeg') 
        else:
            image = cv2.imread('symbols/peace2.jpeg') 
    elif status == METAL:
        if traces:
            print("METAL")
        if hand == 1:
            image = cv2.imread('symbols/metal1.jpeg') 
        else:
            image = cv2.imread('symbols/metal2.jpeg') 
    elif status == FUCK:
        if traces:
            print("****")
        if hand == 1:
            image = cv2.imread('symbols/fuck1.jpeg') 
        else:
            image = cv2.imread('symbols/fuck2.jpeg') 
    elif status == GOOD:
        if traces:
            print("GOOD :)")
        if hand == 1:
            image = cv2.imread('symbols/good1.jpeg') 
        else:
            image = cv2.imread('symbols/good2.jpeg') 
    elif status == INVITA:
        if traces:
            print("INVITA")
        if hand == 1:
            image = cv2.imread('symbols/invita1.jpeg') 
        else:
            image = cv2.imread('symbols/invita2.jpeg') 
    elif status == TEMPLARIOS:
        if traces:
            print("TEMPLARIOS")
        if hand == 1:
            image = cv2.imread('symbols/templarios1.jpeg') 
        else:
            image = cv2.imread('symbols/templarios2.jpeg') 
    else: 
        if hand == 1:
            image = cv2.imread('symbols/hand1.jpeg') 
        else:
            image = cv2.imread('symbols/hand2.jpeg') 

    return image

def print_symbol(left_status, right_status, traces):
    if traces:
        print("Left hand symbol: ")
    img2 = symbol(left_status, hand=2, traces=traces)

    if traces:
        print("Right hand symbol: ")
    img1 = symbol(right_status, hand=1, traces=traces)
    
    image = np.hstack((img1, img2))

    return image


def change_mode(left_status, right_status, mode):
    if(left_status == CHANGE_MODE_1[0] and right_status == CHANGE_MODE_1[1]):
        mode = MOVE_MODE
    elif(left_status == CHANGE_MODE_2[0] and right_status == CHANGE_MODE_2[1]):
        mode = SYMBOLS_MODE
    elif(left_status == CHANGE_MODE_2[0] and right_status == CHANGE_MODE_1[1]):
        mode = FACE_MODE

    return mode
