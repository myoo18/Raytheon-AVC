# Data set

Here is the link to install the data set. use the YOLO8 configuration when downloading
[https://universe.roboflow.com/mqdrones/mq-aruco-2/dataset/3/images?split=valid]

# Environment setup

1. create env

```
python3 -m venv yolov8_env

```

2. activate env (macOS)

```
source yolov8_env/bin/activate
```

3. install ultralytics

```
pip install ultralytics

```

4. install PyTorch

```
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu117
```

# Train YOLO model

1. Run this command to train.

```
yolo detect train \
  data=dataset/data_config.yaml \
  model=yolov8n.pt \
  epochs=50 \
  imgsz=640 \
  name=my_experiment

```

2. strictly testing model detection with webcam

change <code>conf</code> to desired value.

```
yolo detect predict \
    model=runs/detect/my_experiment/weights/best.pt \
    source=0 \
    show=True \
    conf=0.5

```
