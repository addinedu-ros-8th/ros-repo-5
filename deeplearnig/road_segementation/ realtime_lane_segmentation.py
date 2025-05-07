import cv2
from ultralytics import YOLO

# 모델 로드
model = YOLO('/home/vit/dev_ws/road_seg_project/yolov8m-seg200/weights/best.pt')  # 또는 절대경로로 지정

# 웹캠 열기 (0번 카메라)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("❌ 웹캠을 열 수 없습니다.")
    exit()

print("✅ 웹캠 스트리밍 시작 (종료: Q 키)")

while True:
    ret, frame = cap.read()
    if not ret:
        print("⚠️ 프레임을 읽을 수 없습니다.")
        break

    # YOLO 추론 (stream=True로 하면 실시간 처리 최적화됨)
    results = model.predict(source=frame, task='segment', stream=False)

    # 결과 시각화
    annotated_frame = results[0].plot()

    # 출력
    cv2.imshow("YOLOv8 Lane Segmentation", annotated_frame)

    # 'q'를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 종료 처리
cap.release()
cv2.destroyAllWindows()
