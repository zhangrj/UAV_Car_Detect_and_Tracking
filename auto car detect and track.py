import numpy as np
import cv2
import os
import sys
import tensorflow as tf
from object_detection.utils import ops as utils_ops
from utils import label_map_util
from utils import visualization_utils as vis_util


# 模型所在路径
MODEL_NAME = 'ssd_mobilenet_v2_coco_2018_03_29'
# MODEL_NAME = 'ssd_mobilenet_v1_coco_2017_11_17'
# MODEL_NAME = 'car_inference_graph'   

# 用于检测的frozen_inference_graph.pb所在路径
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

# 标签文件所在路径
PATH_TO_LABELS = os.path.join('data', 'mscoco_label_map.pbtxt')
# PATH_TO_LABELS = os.path.join('training', 'object-detection.pbtxt')

# 可检测的目标类别数
# NUM_CLASSES = 90
NUM_CLASSES = 3


# 将.pb文件载入内存
detection_graph = tf.Graph()
with detection_graph.as_default():
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')


# 载入标签文件
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# 读入视频
video = cv2.VideoCapture("test_videos/test3.mp4")
# 检测能否打开视频呢
if not video.isOpened():
  print("Could not open video")
  sys.exit()
# 检测能否读取视频
'''
ok, frame1 = video.read()
if not ok:
    print("Cannot read video file")
    sys.exit()
'''
# 创建KCF跟踪器
tracker = cv2.TrackerKCF_create()

# 读取分辨率
frame_width = int(video.get(3))
frame_height = int(video.get(4))

# 转换目标检测输出为实际像素值
def convert_to_realpx(box_array, frame_height, frame_width):
  ymin = int(box_array[0]*frame_height)
  xmin = int(box_array[1]*frame_width)
  ymax = int(box_array[2]*frame_height)
  xmax = int(box_array[3]*frame_width)
  box = xmin, ymin, xmax, ymax
  return box

# 检测单张图片
def run_inference_for_single_image(image, graph):
  with graph.as_default():
    with tf.Session() as sess:
      # 获取输入输出张量的（tensors）句柄
      ops = tf.get_default_graph().get_operations()
      all_tensor_names = {output.name for op in ops for output in op.outputs}
      tensor_dict = {}
      for key in [
          'num_detections', 'detection_boxes', 'detection_scores',
          'detection_classes', 'detection_masks'
      ]:
        tensor_name = key + ':0'
        if tensor_name in all_tensor_names:
          tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(tensor_name)
      if 'detection_masks' in tensor_dict:
        detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
        detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
        real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
        detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
        detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
        detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
            detection_masks, detection_boxes, image.shape[0], image.shape[1])
        detection_masks_reframed = tf.cast(tf.greater(detection_masks_reframed, 0.5), tf.uint8)
        tensor_dict['detection_masks'] = tf.expand_dims(detection_masks_reframed, 0)
      image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

      # Run inference
      output_dict = sess.run(tensor_dict,
                             feed_dict={image_tensor: np.expand_dims(image, 0)})

      # 所有的输出均为float32 numpy arrays，进行类型转换
      output_dict['num_detections'] = int(output_dict['num_detections'][0])
      output_dict['detection_classes'] = output_dict['detection_classes'][0].astype(np.uint8)
      output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
      output_dict['detection_scores'] = output_dict['detection_scores'][0]
      if 'detection_masks' in output_dict:
        output_dict['detection_masks'] = output_dict['detection_masks'][0]
  return output_dict

# 初始化跟踪器
def init_kcfTracker(frame):
  result = run_inference_for_single_image(frame, detection_graph)
  scroe = result['detection_scores'][0]
  detect_box = result['detection_boxes'][0]
  init_flag = False
  # 置信度大于0.5则初始化
  if scroe>0.5:
    converted_box = convert_to_realpx(detect_box, frame_height, frame_width)
    init_box = (converted_box[0], converted_box[1], converted_box[2]-converted_box[0], converted_box[3]-converted_box[1])
    tracker.init(frame, init_box)
    init_flag = True
  return init_flag
  
# 帧数统计
frame_count = 0
# 保存检测视频
out = cv2.VideoWriter('outpy.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (frame_width, frame_height))

# 寻找第一个出现检测目标的帧并初始化跟踪器
while True:
  ok,  frame = video.read()
  if not ok:
    break
  frame_count = frame_count + 1
  cv2.putText(frame, "Frame : " + str(frame_count), (100, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
  # 写入视频
  out.write(frame)
  cv2.imshow('car track', frame)
  if init_kcfTracker(frame):
    break
  

i = 0
# 每50帧检测一次并重新跟踪
while True:
  ok,  image = video.read()
  i = i+1
  if not ok:
    break
  if (i%50==0):
    tracker = cv2.TrackerKCF_create()
    re_initTracker = init_kcfTracker(image)
  else:
    ok, bbox = tracker.update(image)
    if ok:
      p1 = (int(bbox[0]), int(bbox[1]))
      p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
      cv2.rectangle(image, p1, p2, (255, 0, 0), 2, 1)
    else:
      cv2.putText(image, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

  frame_count = frame_count + 1
  cv2.putText(image, "Frame : " + str(frame_count), (100, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
   # 写入视频
  out.write(image)
  cv2.imshow('car track', image)
  if cv2.waitKey(25) & 0xFF == ord('q'):
    cv2.destroyAllWindows()
    break
