#!/usr/bin/env python3
"""
Trains EfficientDetLite-v0 on a single 'car' class and exports an Edge-TPU-ready
TFLite model + labels.txt.

Where the model is trained
python train_car_edgetpu.py --data_dir car_data --epochs 50

"""

import argparse, pathlib, tflite_model_maker as mm
from tflite_model_maker import object_detector

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--data_dir", required=True, help="car_data/ folder")
    p.add_argument("--epochs", type=int, default=50)
    p.add_argument("--batch_size", type=int, default=8)
    p.add_argument("--tflite", default="car_detector.tflite")
    p.add_argument("--labels", default="labels.txt")
    args = p.parse_args()

    train_data, val_data, test_data = object_detector.DataLoader.from_pascal_voc(
        images_dir      = pathlib.Path(args.data_dir, "images"),
        annotations_dir = pathlib.Path(args.data_dir, "annotations"),
        label_map       = {1: "car"},
        split_ratio     = (0.8, 0.1, 0.1)
    )

    spec  = object_detector.EfficientDetLite0Spec()  # Edge-TPU–compatible backbone
    model = object_detector.create(
        train_data, model_spec=spec, batch_size=args.batch_size,
        epochs=args.epochs, train_whole_model=False, validation_data=val_data
    )

    print("Test mAP:", model.evaluate(test_data))

    # Export plain TFLite + labels file (needed by pycoral)
    model.export(export_dir=".",
                 tflite_filename=args.tflite,
                 label_filename=args.labels,
                 export_format=[mm.config.ExportFormat.TFLITE,
                                mm.config.ExportFormat.LABEL])

if __name__ == "__main__":
    main()
