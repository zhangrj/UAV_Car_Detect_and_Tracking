# 引入所需模块
import cv2
import sys

# 鼠标框选区域，用于显示鼠标轨迹
selection = None
# 框选开始
drag_start = None
# 框选完成区域即跟踪目标
track_window = None
# 跟踪开始标志
track_start = False
# 创建KCF跟踪器
tracker = cv2.TrackerKCF_create()


# 鼠标响应函数
def onmouse(event, x, y, flags, param):
    global selection, drag_start, track_window, track_start
    # 鼠标左键按下
    if event == cv2.EVENT_LBUTTONDOWN:
        drag_start = (x, y)
        track_window = None
    # 开始拖拽
    if drag_start:
        xmin = min(x, drag_start[0])
        ymin = min(y, drag_start[1])
        xmax = max(x, drag_start[0])
        ymax = max(y, drag_start[1])
        selection = (xmin, ymin, xmax, ymax)
    # 鼠标左键弹起
    if event == cv2.EVENT_LBUTTONUP:
        drag_start = None
        selection = None
        track_window = (xmin, ymin, xmax - xmin, ymax - ymin)
        if track_window and track_window[2] > 0 and track_window[3] > 0:
            track_start = True
            # 跟踪器以鼠标左键弹起时所在帧和框选区域为参数初始化
            tracker.init(frame, track_window)


# 读取视频/摄像头
video = cv2.VideoCapture(0)
# video = cv2.VideoCapture("test2.mp4")
# 命名窗口，第二个参数表示窗口可缩放
cv2.namedWindow('KCFTracker', cv2.WINDOW_NORMAL)
# 为窗口绑定鼠标响应函数onmouse
cv2.setMouseCallback('KCFTracker', onmouse)

# 摄像头未正确打开则退出
if not video.isOpened():
    print('Could not open video')
    sys.exit()

while True:
    # 读取当前帧
    ok, frame = video.read()
    if not ok:
        break
    # 以矩形标记鼠标框选区域
    if selection:
        x0, y0, x1, y1 = selection
        cv2.rectangle(frame, (x0, y0), (x1, y1), (255, 0, 0), 2, 1)

    # 函数执行开始时间
    timer = cv2.getTickCount()

    # 更新跟踪器得到最新目标区域
    track_ok = None
    if track_start:
        track_ok, bbox = tracker.update(frame)

    # 计算fps
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)

    # 画出目标最西边边界区域
    # 如果跟踪成功
    if track_ok:
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
    elif not track_start:
        cv2.putText(frame, "No tracking target selected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
    elif not track_ok:
        cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    # 显示提示信息
    cv2.putText(frame, "KCF Tracker", (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
    cv2.putText(frame, "FPS : " + str(int(fps)), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)

    # 显示结果
    cv2.imshow("KCFTracker", frame)

    # 按ESC退出
    k = cv2.waitKey(1) & 0xff
    if k == 27:
        break

video.release()
cv2.destroyAllWindows()
