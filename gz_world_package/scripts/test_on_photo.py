from ultralytics import YOLO

# Укажи путь к твоей обученной cls-модели
model = YOLO("/home/marat/exchange_project_ws/src/collab_industrial_research/gz_world_package/config/camera_models/final_left_best.pt")  # или runs/classify/trainX/weights/best.pt

# Укажи изображение из датасета
img_path = "/home/marat/camera_images/left_camera/safe/freeleft_99_397000000.png"

res = model(img_path, verbose=False)[0]
probs = res.probs  # объект Probs

top_id = int(probs.top1)
top_conf = float(probs.top1conf)

print("Predicted class:", res.names[top_id])
print("Confidence:", top_conf)

# Если хочешь увидеть вероятности по всем классам:
print("All probabilities:")
for i, name in res.names.items():
    p = float(probs.data[i])
    print(f" {i}  {name}: {p:.4f}")
