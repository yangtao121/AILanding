import cv2

capture = cv2.VideoCapture(0)

while True:
    ret, frame = capture.read()
    frame = frame[:, :, 0]
    cv2.imshow('video', frame)
    c = cv2.waitKey(50)
