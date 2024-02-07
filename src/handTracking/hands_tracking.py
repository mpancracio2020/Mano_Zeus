#############################################
## github:    madport                      ##
## github:    vbarcena2020
#############################################

# You have to select how many hands you want and if it is only one wich one


import cv2
import mediapipe as mp
import time
import serial
import sys

PORT = '/dev/ttyACM0'
SERIALBEGIN = 9600
INIT_CHAR = "$"

COLOR_LETTERS = (255, 255, 255)
COLOR_USED_NODES = (0, 255, 0)
COLOR_REFERENCE_NODES = (255, 255, 0)

RATIO_NODES = 4

USED_NODES = (4, 8, 12, 16, 20)
REFERENCE_NODES = (5, 13, 17)

THUMB_FINGER = 4
INDEX_FINGER = 8
MIDDLE_FINGER = 12
RING_FINGER = 16
PINKY_FINGER = 20

REFERENCE_THUMB = 5
REFERENCE_UPPER = 13
REFERENCE_WRIST = 17

class handDetector():
    def __init__(self):
        self.mode = False
        self.maxHands = 1
        self.detectionCon = 1
        self.trackCon = 0.5

        self.pTime_ = 0  # Used to calculate FPS

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils
        

    def findHands(self,img, draw = True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
        return img


    def getNodesPosition(self):

        lmlist = []  # All cordinates of all nodes
        all_points = self.results.multi_hand_landmarks
        n = 0  # Go across the lmlist array

        if all_points:
            myHand = all_points[0]  # We use only one hand
            for landmark in myHand.landmark:
                
                lmlist.append(landmark)
                n += 1

        return lmlist


    def showFps(self, img):
        cTime = time.time()
        fps = 1 / (cTime - self.pTime_)
        self.pTime_ = cTime

        cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, COLOR_LETTERS, 3)


    def showNodes(self, img, cap, nodes, color):
        all_points = self.results.multi_hand_landmarks
        frameWidth = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        frameHeight = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

        if all_points:
            myHand = all_points[0]  # Only there is a one hand
            for node in nodes:
                landmark = myHand.landmark[node]
                pixelCoordinates = self.mpDraw._normalized_to_pixel_coordinates(landmark.x, landmark.y, frameWidth, frameHeight)
                cv2.circle(img, pixelCoordinates, RATIO_NODES, color, -1)
    

    def getFingersPosition_basic_left(self, lmlist):
        thumb, index, middle, ring, pinky, wrist = 0, 1, 2, 3, 4, 5
        position = [1, 1, 1, 1, 1, 1]  # All fingers up

        if lmlist[THUMB_FINGER].x >= lmlist[REFERENCE_THUMB].x:
            position[thumb] = 0

        if lmlist[INDEX_FINGER].y >= lmlist[REFERENCE_UPPER].y:
            position[index] = 0

        if lmlist[MIDDLE_FINGER].y >= lmlist[REFERENCE_UPPER].y:
            position[middle] = 0

        if lmlist[RING_FINGER].y >= lmlist[REFERENCE_UPPER].y:
            position[ring] = 0

        if lmlist[PINKY_FINGER].y >= lmlist[REFERENCE_UPPER].y:
            position[pinky] = 0 

        if lmlist[THUMB_FINGER].x >= lmlist[REFERENCE_WRIST].x:
            position[wrist] = 0

            if lmlist[THUMB_FINGER].x <= lmlist[REFERENCE_THUMB].x:
                position[thumb] = 0
            else:
                position[thumb] = 1

        return position
    
    def getFingersPosition_basic_right(self, lmlist):
        thumb, index, middle, ring, pinky, wrist = 0, 1, 2, 3, 4, 5
        position = [1, 1, 1, 1, 1, 1]  # All fingers up

        if (lmlist[THUMB_FINGER].x <= lmlist[REFERENCE_THUMB].x):
            position[thumb] = 0 

        if (lmlist[INDEX_FINGER].y >= lmlist[REFERENCE_UPPER].y):
            position[index] = 0 

        if (lmlist[MIDDLE_FINGER].y >= lmlist[REFERENCE_UPPER].y):
            position[middle] = 0 

        if (lmlist[RING_FINGER].y >= lmlist[REFERENCE_UPPER].y):
            position[ring] = 0 

        if (lmlist[PINKY_FINGER].y >= lmlist[REFERENCE_UPPER].y):
            position[pinky] = 0 

        if (lmlist[THUMB_FINGER].x <= lmlist[REFERENCE_WRIST].x):
            position[wrist] = 0

            if (lmlist[THUMB_FINGER].x >= lmlist[REFERENCE_THUMB].x):
                position[thumb] = 0
            else:
                position[thumb] = 1
            

        return position

def arrayToString (array):
    string= INIT_CHAR
    for i in array:
        string=string + str(i)

    return string

def usage_error():
    print("Usage: python3 hands_tacking.py <num_hands> (1-2) <hand_wanted> (0-left, 1-right, if num_hands == 1)")
    sys.exit(1)

def main():
    if len(sys.argv) < 2:
        usage_error()

    # Obtener el nombre del archivo de la imagen del primer argumento
    hands = int(sys.argv[1])

    if (hands != 1 and hands != 2):
        usage_error()
    
    hand = 0

    if (hands == 1):
        if (len(sys.argv) != 3):
            usage_error()
        hand = int(sys.argv[2])
    
    

    if (hand != 0 and hand != 1):
        usage_error()

    cap = cv2.VideoCapture(0)
    detector = handDetector()
    detector_left = handDetector()
    detector_right = handDetector()
    # ser = serial.Serial(
    #     port=PORT,
    #     baudrate=SERIALBEGIN,
    #     parity=serial.PARITY_NONE,
    #     stopbits=serial.STOPBITS_ONE,
    #     bytesize=serial.EIGHTBITS,
    #     timeout=1
    # )

    while True:
        success, img = cap.read()
        high, width, _ = img.shape

        # Cortar la mitad izquierda de la imagen
        if (hands == 1):
            img = detector.findHands(img)  # Draws all the nodes and lines
            detector.showFps(img)
            lmlist = detector.getNodesPosition()

            if len(lmlist) != 0:
                if (hand == 0):
                    position = detector.getFingersPosition_basic_left(lmlist)
                elif (hand == 1):
                    position = detector.getFingersPosition_basic_right(lmlist)

                out = arrayToString(position)
                print(out, "\n")
                # serialOut = bytes(out, 'utf-8')
                # ser.write(serialOut)
            
            # Shows the image
            cv2.imshow("Image", img)
            cv2.waitKey(1)

            # Image drawing
        elif (hands == 2):
            img_right = img[:, 0:int(width/2)]
            img_left = img[:, int(width/2):width]

            img_left = detector_left.findHands(img_left)  # Draws all the nodes and lines
            detector_left.showFps(img_left)

            # Fingers position getter and publisher
            lmlist_left = detector_left.getNodesPosition()

            if len(lmlist_left) != 0:
                position = detector_left.getFingersPosition_basic_left(lmlist_left)

                out = arrayToString(position)
                print("left hand: ", out, "\n")
                # serialOut = bytes(out, 'utf-8')
                # ser.write(serialOut)
            
            img_right = detector_right.findHands(img_right)  # Draws all the nodes and lines
            detector_right.showFps(img_right)

            # Fingers position getter and publisher
            lmlist_right = detector_right.getNodesPosition()

            if len(lmlist_right) != 0:
                position = detector_right.getFingersPosition_basic_right(lmlist_right)

                out = arrayToString(position)
                print("rigth hand: ", out, "\n")
                # serialOut = bytes(out, 'utf-8')
                # ser.write(serialOut)
            
            # Shows the image
            cv2.imshow("Image_left", img_left)
            cv2.imshow("Image_right", img_right)
            cv2.waitKey(1)


if __name__ == "__main__":
    main()
