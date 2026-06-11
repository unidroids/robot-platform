yolo export model=best.pt format=engine device=0 half=True imgsz=480,640

yolo export model=cara-single.pt format=engine device=0 half=True imgsz=480,640

yolo export model=cara-batch.pt format=engine half=True device=0 batch=2 imgsz=480,640 