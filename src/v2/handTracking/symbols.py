import cv2
import numpy as np

# SYMBOLS NEADED
MODE_CHANGE_1 = [[1, 1, 0, 0, 0, 1, 1], [1, 1, 0, 0, 0, 0, 1]]
MODE_CHANGE_2 = [[1, 1, 0, 0, 0, 0, 1], [1, 1, 0, 0, 0, 1, 1]]
MOVE = 1
NOT_MOVE = 0
FACE_MODE = 2

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
    if(left_status == MODE_CHANGE_1[0] and right_status == MODE_CHANGE_1[1]):
        mode = MOVE
    elif(left_status == MODE_CHANGE_2[0] and right_status == MODE_CHANGE_2[1]):
        mode = NOT_MOVE
    elif(left_status == MODE_CHANGE_2[0] and right_status == MODE_CHANGE_1[1]):
        mode = FACE_MODE

    return mode
