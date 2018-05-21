# 导入所需模块
import cv2 as cv
import numpy as np
import imutils

# 打开摄像头
cap = cv.VideoCapture(0)

while True:
    # 读取每一帧
    _, frame = cap.read()
    # 重设图片尺寸以提高计算速度
    frame = imutils.resize(frame, width=600)
    # 进行高斯模糊
    blurred = cv.GaussianBlur(frame, (11, 11), 0)
    # 转换颜色空间到HSV
    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
    # 定义红色无图的HSV阈值
    lower_red = np.array([60, 100, 100])
    upper_red = np.array([180, 255, 255])
    # 对图片进行二值化处理
    mask = cv.inRange(hsv, lower_red, upper_red)
    # 腐蚀操作
    mask = cv.erode(mask, None, iterations=2)
    # 膨胀操作，先腐蚀后膨胀以滤除噪声
    mask = cv.dilate(mask, None, iterations=2)

    cv.imshow('mask', mask)

    # 寻找图中轮廓
    cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]

    # 如果存在至少一个轮廓则进行如下操作
    if len(cnts) > 0:
        # 找到面积最大的轮廓
        c = max(cnts, key=cv.contourArea)
        # 使用最小外接圆圈出面积最大的轮廓
        ((x, y), radius) = cv.minEnclosingCircle(c)
        # 计算轮廓的矩
        M = cv.moments(c)
        # 计算轮廓的重心
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # 只处理尺寸足够大的轮廓
        if radius > 5:
            # 画出最小外接圆
            cv.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            # 画出重心
            cv.circle(frame, center, 5, (0, 0, 255), -1)

    cv.imshow('frame', frame)
    k = cv.waitKey(5) & 0xFF
    if k == 27:
        break
cap.release()
cv.destroyAllWindows()
