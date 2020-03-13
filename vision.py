import cv2
import sys
import math
import cv2 as cv
import numpy as np

cap = cv2.VideoCapture(0, cv.CAP_DSHOW)

pt1 = [0,0];
pt2 = [0,0];

while (True):
    ret, src = cap.read()

    src = cv2.resize(src, (640, 480))

    dst = cv.Canny(src, 50, 200, None, 3)

# ROI setup
    imgcut = dst[0:640, 0:480]

    cdst = cv.cvtColor(imgcut, cv.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)

    lines = cv.HoughLines(imgcut, 1, np.pi / 180, 210, None, 0, 0)

    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
            pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
            cv.line(cdst, pt1, pt2, (0, 0, 255), 3, cv.LINE_AA)
    try:
        print("the slope of the line is ", (pt1[1]-pt2[1])/(pt1[0]-pt2[0]))
    except ZeroDivisionError:
        pass

    # line basis
    base_point1 = (150,0);
    base_point2 = (150,1000);
    cv.line(cdst, base_point1, base_point2, (0,255,0), 3, cv.LINE_AA);

    linesP = cv.HoughLinesP(dst, 1, np.pi / 180, 150, None, 100, 10)

    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv.LINE_AA)

    cv.imshow("Source", src)
    cv.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
    cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
