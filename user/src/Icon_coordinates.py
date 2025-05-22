# coordinate_mapper.py
import numpy as np
import cv2                              
from typing import Tuple, Dict

# A =1 D = 2
# R = 3 V = 4
# K = 5 L = 6


ICON_COORDINATES: Dict[str, Tuple[float, float]] = {
    "Icon1": ( 0.262,  0.161),
    "Icon2": ( 0.17,  0.217),
    "Icon3": (-0.015, -0.078),
    "Icon4": (-0.045,  0.021),
    "Icon5": ( 0.08,  0.069),
    "Icon6": ( 0.1,  0.097),
}

PIXEL_COORDINATES = {
    "Icon1": (34, 42),
    "Icon2": (496, 17),
    "Icon3": (16, 325),
    "Icon4": (499, 321),
    "Icon5": (236, 164),
    "Icon6": (274, 165),
}

world2pix_pairs = {}
for name in ICON_COORDINATES:
    world2pix_pairs[ICON_COORDINATES[name]] = PIXEL_COORDINATES[name]



class CoordinateMapper:
    def __init__(self, img_width: int, img_height: int,
                 world2pix_pairs: Dict[Tuple[float, float], Tuple[int, int]], marker_length: float):
        self.src_pts = np.array([
            [0.263, 0.161],
            [0.170, 0.217],
            [-0.015, -0.078]
        ], dtype=np.float32)

        self.dst_pts = np.array([
            [34, 42],
            [496, 17],
            [16, 325]
        ], dtype=np.float32)

        self.affine_matrix = cv2.getAffineTransform(self.src_pts, self.dst_pts)
        self.M = self.affine_matrix  # 핵심! world_to_pixel()에서 이걸 사용함

        print("[DEBUG] CoordinateMapper - Affine matrix:\n", self.M)

    def world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        """실좌표 (m) -> 픽셀 (int)"""
        if self.M is None:
            print("[ERROR] Affine 변환 행렬이 설정되지 않았습니다.")
            return (0, 0)
        
        src = np.array([[x, y, 1]], dtype=np.float32).T   # 3×1
        dst = self.M @ src                                # 2×1
        return int(dst[0,0]), int(dst[1,0])

    def calculate_affine_matrix(self):
        if len(self.world2pix_pairs) < 3:
            print("[ERROR] 3개 이상의 매핑쌍 필요")
            return None
        src_pts = np.float32(list(self.world2pix_pairs.keys()))
        dst_pts = np.float32(list(self.world2pix_pairs.values()))
        print("[DEBUG] src_pts:", src_pts)
        print("[DEBUG] dst_pts:", dst_pts)
        M, _ = cv2.estimateAffine2D(src_pts, dst_pts)
        print("[DEBUG] Affine matrix:\n", M)
        if M is not None:
            print("[INFO] 수동 매핑 기반 Affine 변환 계산 완료")

        else:
            print("[ERROR] Affine 변환 계산 실패")
        
        return M

