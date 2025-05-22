import cv2

# 이미지 파일 경로
img_path = "/home/lim/git/ros-repo-5/user/src/data/map/map.png"  # 이미지 경로를 실제로 맞게 수정!

# 이미지 읽기
img = cv2.imread(img_path)
if img is None:
    print("이미지 파일을 찾을 수 없습니다.")
    exit()

# 클릭 시 좌표를 출력하는 콜백 함수
def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Clicked pixel: ({x}, {y})")
        # 클릭한 위치에 동그라미 표시 (시각적으로도 확인 가능)
        cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow("map", img)

# 창 생성 및 콜백 연결
cv2.imshow("map", img)
cv2.setMouseCallback("map", click_event)

print("이미지 창을 클릭하면 좌표가 콘솔에 출력됩니다.")
cv2.waitKey(0)
cv2.destroyAllWindows()
