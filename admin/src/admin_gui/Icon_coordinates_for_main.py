import numpy as np
import cv2
from typing import Tuple, Dict

# A =1 D = 2
# R = 3 V = 4
# K = 5 L = 6


ICON_COORDINATES : Dict[str, Tuple[float, float]]= {
    "A": (0.262, 0.161), "B": (0.241, 0.196), "C": (0.215, 0.229), "D": (0.17, 0.217), "E": (0.177, 0.192),
    "F": (0.205, 0.127), "G": (0.183, 0.146), "H": (0.139, 0.16), "I": (0.102, 0.148),
    "J": (0.127, 0.057), "K": (0.08, 0.069), "L": (0.1, 0.097), "M": (0.039, 0.094),
    "N": (0.063, 0.005), "O": (0.031, 0.002), "P": (-0.013, 0.015), "Q": (-0.022, 0.033),
    "R": (-0.015, -0.078), "S": (-0.036, -0.055), "T": (-0.005, -0.017), "U": (-0.055, -0.018), "V": (-0.045, 0.021)
}

# 961*521
PIXEL_COORDINATES  = {
    "A":(108,87),
    "B":(332,47),
    "C":(622, 26),
    "D":(838, 52),
    "E" :(633,83),
    "F":(158, 134),
    "G":(342, 106),
    "H":(640, 138),
    "I":(788,158),
    "J":(140, 230),
    "K":(442, 235),
    "L":(503, 241),
    "M":(820, 267),
    "N":(162, 336),
    "O":(352, 357),
    "P":(618, 395),
    "Q":(798, 377),
    "R":(109, 465),
    "S":(319, 475),
    "T":(328, 412),
    "U":(631, 454),
    "V":(871, 412)
     
}



# 아이콘 순서를 명시적으로 고정
ICON_ORDER = ["A", "D", "R", "V", "K", "L"]




class CoordinateMapper:
    def __init__(self, img_width: int, img_height: int, marker_length: float):
        self.img_width = img_width
        self.img_height = img_height
        self.marker_length = marker_length
        self.outlier_log = []  # 튐 좌표 로그

        self.src_pts = np.float32([ICON_COORDINATES[name] for name in ICON_ORDER])
        self.dst_pts = np.float32([PIXEL_COORDINATES[name] for name in ICON_ORDER])

        self.M = self.calculate_affine_matrix()
        print("[DEBUG] CoordinateMapper - Affine matrix:\n", self.M)

    def calculate_affine_matrix(self):
        print("[DEBUG] src_pts:", self.src_pts)
        print("[DEBUG] dst_pts:", self.dst_pts)
        M, _ = cv2.estimateAffine2D(self.src_pts, self.dst_pts, method=cv2.LMEDS)
        if M is not None:
            print("[DEBUG] estimateAffine2D 결과:\n", M)
        else:
            print("[ERROR] estimateAffine2D 실패")
        return M

    def world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        if self.M is None:
            print("[ERROR] Affine 변환 행렬이 설정되지 않았습니다.")
            return (0, 0)
        src = np.array([[x, y, 1]], dtype=np.float32).T
        dst = self.M @ src
        px, py = int(dst[0, 0]), int(dst[1, 0])
        return px, py

# ---- 디버깅용: 변환 검증 ----
if __name__ == "__main__":
    mapper = CoordinateMapper(961, 521, marker_length=0.1)
    print("\n[실좌표 → 픽셀 변환 검증 결과]")
    for name in ICON_COORDINATES:
        real_x, real_y = ICON_COORDINATES[name]
        px, py = mapper.world_to_pixel(real_x, real_y)
        ref_px, ref_py = PIXEL_COORDINATES[name]
        print(f"{name}: world({real_x:.3f}, {real_y:.3f}) → pixel({px}, {py}) | 기준: ({ref_px}, {ref_py}) | 오차: ({px-ref_px}, {py-ref_py})")