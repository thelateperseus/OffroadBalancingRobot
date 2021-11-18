import cv2
import numpy as np
import sys
import tensorflow as tf
import time

#tf.get_logger().setLevel('ERROR')

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils


def apply_non_max_suppression(detections, iou_thresh=0.5, score_thresh=0.6):
    """
    Function to apply non-maximum suppression on different classes
    Parameters
    ----------
    detections : dictionary
        dictionary containing:
            'detection_boxes' : Bounding boxes coordinates. Shape (N, 4)
            'detection_classes' : Class indices detected. Shape (N)
            'detection_scores' : Shape (N)
            'num_detections' : Total number of detections i.e. N. Shape (1)
    iou_thresh : int, optional
        Intersection Over Union threshold value. The default is 0.5.
    score_thresh : int, optional
        Score threshold value below which to ignore. The default is 0.6.
    Returns
    -------
    detections : dictionary
        dictionary containing only scores and IOU greater than threshold.
            'detection_boxes' : Bounding boxes coordinates. Shape (N2, 4)
            'detection_classes' : Class indices detected. Shape (N2)
            'detection_scores' : Shape (N2)
            where N2 is the number of valid predictions after those conditions.
    """

    q = 90 # no of classes in model
    num = int(detections['num_detections'])
    boxes = np.zeros([1, num, q, 4])
    scores = np.zeros([1, num, q])
    # val = [0]*q
    for i in range(num):
        # indices = np.where(classes == output_dict['detection_classes'][i])[0][0]
        boxes[0, i, detections['detection_classes'][i]-1, :] = detections['detection_boxes'][i]
        scores[0, i, detections['detection_classes'][i]-1] = detections['detection_scores'][i]

    nmsd = tf.image.combined_non_max_suppression(
        boxes=boxes,
        scores=scores,
        max_output_size_per_class=num,
        max_total_size=num,
        iou_threshold=iou_thresh,
        score_threshold=score_thresh,
        pad_per_class=False,
        clip_boxes=False)
    valid = nmsd.valid_detections[0].numpy()
    return {
        'detection_boxes' : nmsd.nmsed_boxes[0].numpy()[:valid],
        'detection_classes' : nmsd.nmsed_classes[0].numpy().astype(np.int64)[:valid],
        'detection_scores' : nmsd.nmsed_scores[0].numpy()[:valid],
    }


PATH_TO_LABELS = './exported-models/ssd_mobilenet_v2_320x320_ball/ball_label_map.pbtxt'
category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS, use_display_name=True)

print('Loading model...', end='')
start_time = time.time()

# Load saved model and build the detection function
#PATH_TO_SAVED_MODEL = './model/ssd_resnet50_v1_fpn_640x640_coco17_tpu-8/saved_model'
PATH_TO_SAVED_MODEL = './exported-models/ssd_mobilenet_v2_320x320_ball/saved_model'
detect_fn = tf.saved_model.load(PATH_TO_SAVED_MODEL)

end_time = time.time()
elapsed_time = end_time - start_time
print(f'Done! Took {elapsed_time} seconds')

cap = cv2.VideoCapture(sys.argv[1])
codec = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter("tracking-output.avi", codec, 25, (320,240))

frameCount = 0
start_time = time.time()
while(True):
    ret, image = cap.read()
    if not ret:
        break

    #image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    #image_rgb = cv2.resize(image_rgb, (640, 640), cv2.INTER_AREA)
    #image_rgb = image_rgb.reshape([1, 640, 640, 3])

    # The input needs to be a tensor, convert it using `tf.convert_to_tensor`.
    input_tensor = tf.convert_to_tensor(image)
    # The model expects a batch of images, so add an axis with `tf.newaxis`.
    input_tensor = input_tensor[tf.newaxis, ...]

    detections = detect_fn(input_tensor)

    # All outputs are batches tensors.
    # Convert to numpy arrays, and take index [0] to remove the batch dimension.
    # We're only interested in the first num_detections.
    num_detections = int(detections.pop('num_detections'))
    detections = {key: value[0, :num_detections].numpy()
                   for key, value in detections.items()}
    detections['num_detections'] = num_detections

    # detection_classes should be ints.
    detections['detection_classes'] = detections['detection_classes'].astype(np.int64)

    detection_boxes = detections['detection_boxes']
    detection_classes = detections['detection_classes']
    detection_scores = detections['detection_scores']

    # detections = apply_non_max_suppression(detections, iou_thresh=0.3, score_thresh=0.4)

    # Remove anything other than sports ball
    # keep = np.ones(detection_classes.shape, dtype=bool)
    # for idx, val in enumerate(detection_classes):
    #     if val != 37: # 37 = sports ball
    #         keep[idx] = False
    # detection_boxes = detection_boxes[keep]
    # detection_classes = detection_classes[keep]
    # detection_scores = detection_scores[keep]

    viz_utils.visualize_boxes_and_labels_on_image_array(
        image,
        detection_boxes,
        detection_classes,
        detection_scores,
        category_index,
        use_normalized_coordinates=True,
        max_boxes_to_draw=200,
        min_score_thresh=.30,
        agnostic_mode=False)

    out.write(image)
    frameCount +=1 
    #cv2.imshow("image", image)
    #key = cv2.waitKey(1) & 0xFF
    #if key == ord("q"):
    #    break

end_time = time.time()
elapsed_time = end_time - start_time
print(f'Done! {frameCount} frames took {elapsed_time} seconds, or {frameCount/elapsed_time} fps')

out.release()
cap.release()
cv2.destroyAllWindows()
