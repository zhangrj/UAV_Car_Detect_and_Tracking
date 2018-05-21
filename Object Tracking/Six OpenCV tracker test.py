import cv2
import sys

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

# 设置跟踪器类型
tracker_types = ['BOOSTING', 'MIL', 'MOSSE', 'KCF', 'TLD', 'MEDIANFLOW']
tracker_type = tracker_types[3]
# 创建跟踪器
if int(minor_ver) < 3:
    tracker = cv2.Tracker_create(tracker_type)
else:
    if tracker_type == 'BOOSTING':
        tracker = cv2.TrackerBoosting_create()
    if tracker_type == 'MIL':
        tracker = cv2.TrackerMIL_create()
    if tracker_type == 'KCF':
        tracker = cv2.TrackerKCF_create()
    if tracker_type == 'TLD':
        tracker = cv2.TrackerTLD_create()
    if tracker_type == 'MEDIANFLOW':
        tracker = cv2.TrackerMedianFlow_create()
    if tracker_type == 'MOSSE':
        tracker = cv2.TrackerMOSSE_create()

# 读取视频
video = cv2.VideoCapture("test3.mp4")

# 无法开打报错退出
if not video.isOpened():
    print("Could not open video")
    sys.exit()

# 读取第一帧
ok, frame = video.read()
if not ok:
    print("Cannot read video file")
    sys.exit()

# 框选待跟踪目标
bbox = cv2.selectROI(frame, False)

# 使用第一帧和框选目标初始化跟踪器
ok = tracker.init(frame, bbox)

# 统计总帧数
frame_count = 0

# 统计总fps
fps_count = 0

while True:
    # 读取下一帧
    ok, frame = video.read()
    if not ok:
        break

    frame_count = frame_count + 1
    # 开启计时器
    timer = cv2.getTickCount()
    # 更新跟踪器
    ok, bbox = tracker.update(frame)
    # 计算fps
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
    fps_count = fps_count + fps
    # 跟踪成功画出边界框，失败显示提示信息
    if ok:
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
    else:
        cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    # 显示相关提示信息
    cv2.putText(frame, tracker_type + " Tracker", (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
    cv2.putText(frame, "FPS : " + str(int(fps)), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
    cv2.putText(frame, "Frame : " + str(frame_count), (100, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)

    # 显示跟踪结果
    cv2.imshow("Tracking", frame)

    # 按ESC退出
    k = cv2.waitKey(1) & 0xff
    if k == 27:
        break

# 计算并显示平均fps
fps_average = int(fps_count/frame_count)
print(frame_count)
print(fps_average)
