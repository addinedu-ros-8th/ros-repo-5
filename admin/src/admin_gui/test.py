import cv2

points = []

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Clicked at: ({x}, {y})")
        points.append((x, y))

# 이미지 불러오기
img = cv2.imread("/home/lim/git/ros-repo-5/admin/src/admin_gui/data/resized_new_map_image.png")  # 파일명에 맞게 수정

cv2.namedWindow("Map")
cv2.setMouseCallback("Map", mouse_callback)

while True:
    temp = img.copy()
    for (x, y) in points:
        cv2.circle(temp, (x, y), 5, (0, 0, 255), -1)
    cv2.imshow("Map", temp)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):  # q 누르면 종료
        break

cv2.destroyAllWindows()

print("\n--- 클릭한 픽셀 좌표 ---")
for pt in points:
    print(pt)
