import pyttsx3
import datetime
import speech_recognition as sr
import wikipedia
import webbrowser
import os
import smtplib
from pynput.mouse import Button, Controller
import pyautogui
from time import sleep
import cv2
import numpy as np
import wx

engine = pyttsx3.init('sapi5')
voices = engine.getProperty('voices')
engine.setProperty('voice', voices[0].id)


def speak(audio):
    engine.say(audio)
    engine.runAndWait()

def wishMe():
    hour = int(datetime.datetime.now().hour)
    if hour >= 0 and hour < 12:
        speak("Good Morning sir")

    elif hour >=12 and hour < 18 :
        speak("Good Afternoon sir")

    else:
        speak("Good Evening sir")
    speak("initializing dependencies and calibrating virtual environment")
    sleep(1)
    speak("we're all set for now!")

def takeCommand():
    #It takes microphone input from the user and returns string output

    r = sr.Recognizer()
    with sr.Microphone() as source:
        r.adjust_for_ambient_noise(source, duration=1)
        print("Listening...")
        audio = r.listen(source)
        try:
            print("Recognizing...")
            query = r.recognize_google(audio, language='en-in')  # Using google for voice recognition.
            print(f"Mr Stark says: {query}\n")  # User query will be printed.

        except Exception as e:
            # print(e)
            print("Say that again please...")  # Say that again will be printed in case of improper voice
            return "None"  # None string will be returned
        return query

def sendEmail(to, content):
    EMAIL_ADDRESS = "your email here"
    EMAIL_PASSWORD = "your password here"

    server = smtplib.SMTP('smtp.gmail.com', 587)
    server.ehlo()
    server.starttls()
    server.login(EMAIL_ADDRESS, EMAIL_PASSWORD)
    server.sendmail("your email here", to, content)
    server.close()

def autoMouse():
    mouse = Controller()

    app = wx.App(False)
    (sx, sy) = wx.GetDisplaySize()
    (camx, camy) = (250, 250)
    cam = cv2.VideoCapture(0)
    cam.set(3, camx)
    cam.set(4, camy)
    lowerBound = np.array([33, 80, 40])
    upperBound = np.array([102, 255, 255])
    kernelOpen = np.ones((5, 5))
    kernelClose = np.ones((20, 20))

    mLoc01d = np.array([0, 0])
    mouseLoc = np.array([0, 0])
    DampingFactor = 2  # should more than 1

    pinchFlag = 0
    openx, openy, openw, openh = (0, 0, 0, 0)

    # mouseLoc =mLoc01d+(targetLoc-mLoc01d)/DampingFactor

    while True:
        ret, img = cam.read()
        # img = cv2.resize(img, (340, 220))

        # convert BGR to HSV
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # create the mask
        mask = cv2.inRange(imgHSV, lowerBound, upperBound)
        # noise removal
        maskOpen = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernelOpen)
        maskClose = cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, kernelClose)
        maskFinal = maskClose
        conts, h = cv2.findContours(maskFinal.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if (len(conts) == 2):
            if (pinchFlag == 1):
                pinchFlag = 0
                mouse.release(Button.left)
            mouse.release(Button.left)
            x1, y1, w1, h1 = cv2.boundingRect(conts[0])
            x2, y2, w2, h2 = cv2.boundingRect(conts[1])
            cv2.rectangle(img, (x1, y1), (x1 + w1, y1 + h1), (255, 0, 0), 2)
            cv2.rectangle(img, (x2, y2), (x2 + w2, y2 + h2), (255, 0, 0), 2)
            cx1 = int(x1 + w1 / 2)
            cy1 = int(y1 + h1 / 2)
            cx2 = int(x2 + w2 / 2)
            cy2 = int(y2 + h2 / 2)
            cx = int((cx1 + cx2) / 2)
            cy = int((cy1 + cy2) / 2)
            cv2.line(img, (cx1, cy1), (cx2, cy2), (255, 0, 0), 2)
            cv2.circle(img, (cx, cy), 2, (0, 0, 255), 2)
            mouseLoc = mLoc01d + ((cx, cy) - mLoc01d) / DampingFactor
            mouse.position = (sx - (mouseLoc[0] * sx / camx), mouseLoc[1] * sy / camy)
            while mouse.position != (sx - (mouseLoc[0] * sx / camx), mouseLoc[1] * sy / camy):
                break
            mLoc01d = mouseLoc
            openx, openy, openw, openh = cv2.boundingRect(
                np.array([[x1, y1], [x1 + w1, y1 + h1], [x2, y2], [x2 + w2, y2 + h2]]))
            # cv2.rectangle(img, (openx,openy), (openx+openw, openy+openh), (255, 0, 0), 2)



        elif (len(conts) == 1):
            x, y, w, h = cv2.boundingRect(conts[0])
            if (pinchFlag == 0):
                if (abs((w * h - openw * openh) * 100 / (w * h)) < 20):
                    pinchFlag = 1
                    mouse.click(Button.left, 2)
                    openx, openy, openw, openh = (0, 0, 0, 0)
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cx = int(x + w / 2)
            cy = int(y + h / 2)
            cv2.circle(img, (cx, cy), int((w + h) / 4), (0, 0, 255), 2)
            mouseLoc = mLoc01d + ((cx, cy) - mLoc01d) / DampingFactor
            mouse.position = (sx - (mouseLoc[0] * sx / camx), mouseLoc[1] * sy / camy)
            while mouse.position != (sx - (mouseLoc[0] * sx / camx), mouseLoc[1] * sy / camy):
                break
            mLoc01d = mouseLoc

        # cv2.imshow("maskClose", maskClose)
        # cv2.imshow("maskOpen", maskOpen)
        # cv2.imshow("mask", mask)
        cv2.imshow("cam", img)
        cv2.waitKey(1)

    video_capture.release()
    cv2.destroyAllWindows()

def handGesture():
    import cv2
    import numpy as np
    import math

    # camera open
    capture = cv2.VideoCapture(0)

    while capture.isOpened():
        # capture frame from camera
        ret, frame = capture.read()
        # make rectangle frame
        cv2.rectangle(frame, (100, 100), (300, 300), (0, 255, 0), 0)
        crop_image = frame[100:300, 100:300]

        # apply gausian blur
        blur = cv2.GaussianBlur(crop_image, (3, 3), 0)

        # change color-space from BGR->HSV
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        # create a masking image with where white will be skin color and rest is black

        mask2 = cv2.inRange(hsv, np.array([2, 0, 0]), np.array([20, 255, 255]))

        # kernel for morphological transformation
        kernel = np.ones((5, 5))

        # Apply morphological transformation to filter out the background noise

        dilation = cv2.dilate(mask2, kernel, iterations=1)
        erosion = cv2.erode(dilation, kernel, iterations=1)

        # apply gaussian blur and threshold
        filtered = cv2.GaussianBlur(erosion, (3, 3), 0)
        ret, thresh = cv2.threshold(filtered, 127, 255, 0)

        # show the threshold image
        cv2.imshow("Threshold", thresh)

        # find contours

        contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # find contour with maximum area
        contour = max(contours, key=lambda x: cv2.contourArea(x))
        # create boundry rectangle around contour

        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(crop_image, (x, y), (x + w, y + h), (0, 0, 255), 0)

        # find convex hull
        hull = cv2.convexHull(contour)

        # draw contour

        drawing = np.zeros(crop_image.shape, np.uint8)
        cv2.drawContours(drawing, [contour], -1, (0, 255, 0), 0)
        cv2.drawContours(drawing, [hull], -1, (0, 0, 255), 0)

        # find convexity defects
        hull = cv2.convexHull(contour, returnPoints=False)
        defects = cv2.convexityDefects(contour, hull)

        # using cosine rule to find defects from start point to end point

        count_defects = 0

        for i in range(defects.shape[0]):
            s, e, f, d = defects[i, 0]
            start = tuple(contour[s][0])
            end = tuple(contour[e][0])
            far = tuple(contour[f][0])

            a = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
            b = math.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
            c = math.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)

            angle = (math.acos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c)) * 180) / 3.14

            # if angle > 90 draw circle at far point

            if angle <= 90:
                count_defects += 1
                cv2.circle(crop_image, far, 1, [0, 0, 255], -1)

            cv2.line(crop_image, start, end, [0, 255, 0], 2)

        # print no of fingers

        if count_defects == 0:
            cv2.putText(frame, "one", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
        elif count_defects == 1:
            cv2.putText(frame, "recognized", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
        elif count_defects == 2:
            cv2.putText(frame, "three", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
        elif count_defects == 3:
            cv2.putText(frame, "four", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
        elif count_defects == 4:
            cv2.putText(frame, "five", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
            speak("The party begins")
            music = 'E:\music'
            hack = os.listdir(music)
            os.startfile(os.path.join(music, hack[0]))
            break

        else:
            pass

        # show required images

        cv2.imshow("Gesture", frame)
        all_image = np.hstack((drawing, crop_image))
        cv2.imshow('Contours', all_image)

        # close the camera if q is pressed
        if cv2.waitKey(1) == ord('q'):
            break

    capture.release()
    cv2.destroyAllWindows()





if __name__ == "__main__":
    wishMe()
    while True:

        query = takeCommand().lower() #Converting user query into lower case

        # Logic for executing tasks based on query
        if 'wikipedia' in query:  #if wikipedia found in the query then this block will be executed
            speak('Searching Wikipedia...')
            query = query.replace("wikipedia", "")
            results = wikipedia.summary(query, sentences=2)
            speak("According to Wikipedia")
            print(results)
            speak(results)

        elif 'open youtube' in query:
            webbrowser.open('youtube.com')

        elif 'open google' in query:
            chrome_path = "C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe"
            webbrowser.register('chrome', None, webbrowser.BackgroundBrowser(chrome_path))
            webbrowser.get('chrome').open_new('www.google.com')

        elif 'open stackoverflow' in query:
            webbrowser.open('stackoverflow.com')

        elif 'initiate wi-fi hacking ' in query:
            wifi_hack = 'C:\\Users\\user\\PycharmProjects\\wifimails'
            hack = os.listdir(wifi_hack)
            os.startfile(os.path.join(wifi_hack, hack[1]))

        elif 'the time' in query:
            strTime = datetime.datetime.now().strftime("%H:%M:%S")
            speak(f"The time now is {strTime}")

        elif 'how are you doing' in query:
            play_video = 'F:\\F.R.I.E.N.D.S\\Season 1'
            video = os.listdir(play_video)
            os.startfile(os.path.join(play_video, video[2]))

        elif 'email to ' in query:
            try:
                speak("what should the content be?")
                content = takeCommand()
                to = "email here"
                sendEmail(to, content)
                speak("Email has been sent!")
            except Exception as e:
                print(e)
                speak("Sorry there might be some issue with the cloud")

        elif "h a c k" in query:
            speak('initiating kali destro')
            virtual_box = 'C:\\ProgramData\\Microsoft\\Windows\\Start Menu\\Programs\\Oracle VM VirtualBox'
            vm = os.listdir(virtual_box)
            os.startfile(os.path.join(virtual_box, vm[1]))
            sleep(2)
            mouse = Controller()
            mouse.position = (818, 281)
            sleep(2)
            mouse.click(Button.left)

        elif "virtual" in query:
            speak("initiating hands free mode")
            speak("virtual environment calibrated")
            autoMouse()



        elif "fun" in query:
            speak("of course sir")
            handGesture()


        elif "thank you" in query:
            speak('my pleasure sir')
            speak("i'll be here if you want anything else")
            exit()






