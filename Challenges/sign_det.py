# TODO: Implement sign_det
"""
Training script:
"""
import numpy as np
import os

from tflite_model_maker.config import ExportFormat
from tflite_model_maker import model_spec
from tflite_model_maker import object_detector

import tensorflow as tf
assert tf.__version__.startswith('2')

tf.get_logger().setLevel('ERROR')
from absl import logging
logging.set_verbosity(logging.ERROR)
# Using Custom Dataset ### CHANGE ME IF NEEDED ###
print(os.getcwd())
dataset_path = "../../dataset/name_of_dataset"
images_path = dataset_path + "/images"
labels_path = dataset_path + "/Annotations"

# Label Map
label_map = {1: 'name_of_object'}

print(f"Images: {images_path}")
print(f"Labels: {labels_path}")
import os
import random
import shutil

def split_dataset(images_path, annotations_path, val_split, test_split, out_path):
  """Splits a directory of sorted images/annotations into training, validation, and test sets.

  Args:
    images_path: Path to the directory with your images (JPGs).
    annotations_path: Path to a directory with your VOC XML annotation files,
      with filenames corresponding to image filenames. This may be the same path
      used for images_path.
    val_split: Fraction of data to reserve for validation (float between 0 and 1).
    test_split: Fraction of data to reserve for test (float between 0 and 1).
  Returns:
    The paths for the split images/annotations (train_dir, val_dir, test_dir)
  """
  _, dirs, _ = next(os.walk(images_path))

  train_dir = os.path.join(out_path, 'train')
  val_dir = os.path.join(out_path, 'validation')
  test_dir = os.path.join(out_path, 'test')

  IMAGES_TRAIN_DIR = os.path.join(train_dir, 'images')
  IMAGES_VAL_DIR = os.path.join(val_dir, 'images')
  IMAGES_TEST_DIR = os.path.join(test_dir, 'images')
  os.makedirs(IMAGES_TRAIN_DIR, exist_ok=True)
  os.makedirs(IMAGES_VAL_DIR, exist_ok=True)
  os.makedirs(IMAGES_TEST_DIR, exist_ok=True)

  ANNOT_TRAIN_DIR = os.path.join(train_dir, 'annotations')
  ANNOT_VAL_DIR = os.path.join(val_dir, 'annotations')
  ANNOT_TEST_DIR = os.path.join(test_dir, 'annotations')
  os.makedirs(ANNOT_TRAIN_DIR, exist_ok=True)
  os.makedirs(ANNOT_VAL_DIR, exist_ok=True)
  os.makedirs(ANNOT_TEST_DIR, exist_ok=True)

  # Get all filenames for this dir, filtered by filetype
  filenames = os.listdir(os.path.join(images_path))
  filenames = [os.path.join(images_path, f) for f in filenames if (f.endswith('.jpg'))]
  # Shuffle the files, deterministically
  filenames.sort()
  random.seed(42)
  random.shuffle(filenames)
  # Get exact number of images for validation and test; the rest is for training
  val_count = int(len(filenames) * val_split)
  test_count = int(len(filenames) * test_split)
  for i, file in enumerate(filenames):
    source_dir, filename = os.path.split(file)
    annot_file = os.path.join(annotations_path, filename.replace("jpg", "xml"))
    if i < val_count:
      shutil.copy(file, IMAGES_VAL_DIR)
      shutil.copy(annot_file, ANNOT_VAL_DIR)
    elif i < val_count + test_count:
      shutil.copy(file, IMAGES_TEST_DIR)
      shutil.copy(annot_file, ANNOT_TEST_DIR)
    else:
      shutil.copy(file, IMAGES_TRAIN_DIR)
      shutil.copy(annot_file, ANNOT_TRAIN_DIR)
  return (train_dir, val_dir, test_dir)
def clean_xml_declaration(annotations_dir):
    for filename in os.listdir(annotations_dir):
        if filename.endswith('.xml'):
            path = os.path.join(annotations_dir, filename)
            with open(path, 'rb') as f:
                content = f.read()
            # Remove XML declaration if present
            if content.startswith(b'<?xml'):
                first_line_end = content.find(b'?>') + 2
                content = content[first_line_end:].lstrip()
            with open(path, 'wb') as f:
                f.write(content)
# Split data into train, validation, test
train_dir, val_dir, test_dir = split_dataset(images_path, labels_path, val_split=0.2, test_split=0.2, out_path='split-dataset')

# Clean data coming in from Label Studio
clean_xml_declaration(os.path.join(train_dir, 'annotations'))
clean_xml_declaration(os.path.join(val_dir, 'annotations'))
clean_xml_declaration(os.path.join(test_dir, 'annotations'))

train_data = object_detector.DataLoader.from_pascal_voc(
    os.path.join(train_dir, 'images'),
    os.path.join(train_dir, 'annotations'), label_map=label_map)

validation_data = object_detector.DataLoader.from_pascal_voc(
    os.path.join(val_dir, 'images'),
    os.path.join(val_dir, 'annotations'), label_map=label_map)

test_data = object_detector.DataLoader.from_pascal_voc(
    os.path.join(test_dir, 'images'),
    os.path.join(test_dir, 'annotations'), label_map=label_map)
# Select a model
spec = object_detector.EfficientDetLite0Spec()
# Train the model
model = object_detector.create(train_data=train_data, 
                               model_spec=spec, 
                               validation_data=validation_data, 
                               epochs=50, 
                               batch_size=10, 
                               train_whole_model=True)
metrics = model.evaluate(test_data)
import matplotlib.pyplot as plt

# Clean label names
labels = [k for k in metrics if not k.startswith('AP_/')]
values = [metrics[k] * 100 for k in labels]  # Convert to %

plt.figure(figsize=(10, 5))
plt.bar(labels, values)
plt.ylabel('Metric (%)')
plt.title('Model Evaluation Metrics (COCO)')
plt.xticks(rotation=45)
plt.grid(axis='y')
plt.tight_layout()
plt.show()
TFLITE_FILENAME = 'name_of_model.tflite' # Rename output here!
LABELS_FILENAME = 'objects.txt'

model.export(export_dir='.', tflite_filename=TFLITE_FILENAME, label_filename=LABELS_FILENAME,
             export_format=[ExportFormat.TFLITE, ExportFormat.LABEL])
# Evaluate tflite file using test data
model.evaluate_tflite(TFLITE_FILENAME, test_data)



"""
Deployment Script:
"""
import cv2
import os
import time

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference

# Define paths to model and label directories
default_path = 'models' # location of model weights and labels
model_name = 'efficientdet0_edgetpu.tflite'
label_name = 'labels.txt'

model_path = default_path + "/" + model_name
label_path = default_path + "/" + label_name

# Define thresholds and number of classes to output
SCORE_THRESH = 0.1
NUM_CLASSES = 1
# [FUNCTION] Modify image to label objs and score
def append_objs_to_img(cv2_im, inference_size, objs, labels):
    height, width, channels = cv2_im.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]
    for obj in objs:
        if obj.score > 0.5:
            bbox = obj.bbox.scale(scale_x, scale_y)
            x0, y0 = int(bbox.xmin), int(bbox.ymin)
            x1, y1 = int(bbox.xmax), int(bbox.ymax)
    
            percent = int(100 * obj.score)
            label = '{}% {}'.format(percent, labels.get(obj.id, obj.id))
    
            cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2_im = cv2.putText(cv2_im, label, (x0, y0+30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
    return cv2_im
# Display Dependencies
import IPython
import IPython.display
from io import BytesIO
import PIL.Image

# [FUNCTION] Prepare .ipynb display
def show_rgb_image_to_display(image_rgb, display):
    """
    Displays a color image in the Jupyter Notebook.
    Assumes image is in RGB format.
    """
    io = BytesIO()
    image = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2RGB)
    PIL.Image.fromarray(image).save(io, 'jpeg')
    img_display = IPython.display.Image(data=io.getvalue())
    display.update(img_display)
# The actual display
display = IPython.display.display('', display_id=1)
# STEP 1: Load model and labels using pycoral.utils
print('Loading {} with {} labels.'.format(model_path, label_path))
interpreter = make_interpreter(model_path)
interpreter.allocate_tensors()
labels = read_label_file(label_path)
inference_size = input_size(interpreter)
# STEP 2: Open webcam
cap = cv2.VideoCapture(0) # Default webcam has ID of 0

# STEP 3: Loop through webcam camera stream and run model
while cap.isOpened():
    ret, frame = cap.read() # Read from webcam
    
    if frame is None:
        break # stop script if frame is empty
    else:
        
        # STEP 4: Preprocess image to the size and shape accepted by model
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb_image = cv2.resize(rgb_image, inference_size)

        # STEP 5: Let the model do the work
        run_inference(interpreter, rgb_image.tobytes())

        # STEP 6: Get objects detected from the model
        objs = get_objects(interpreter, SCORE_THRESH)[:NUM_CLASSES]

        # STEP 7: Label detected objects to frame
        image = append_objs_to_img(frame, inference_size, objs, labels)

        # STEP 8: Show labeled image to screen
        show_rgb_image_to_display(image, display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()
