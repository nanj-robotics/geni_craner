#!/usr/bin/env python3
"""
验证手眼标定：把base下的3D点投影到相机像素
用法: python3 verify_calib.py <x> <y> <z>
"""
import sys
import yaml
import numpy as np
from pathlib import Path

FX = 409.58734130859375
FY = 409.72186279296875
CX = 423.8803405761719
CY = 262.1623840332031
WIDTH = 848
HEIGHT = 530

def quat_to_rotmat(q):
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]
    ])

def main():
    if len(sys.argv) < 4:
        print("用法: python3 verify_calib.py <x> <y> <z>")
        sys.exit(1)
    point_base = np.array([float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])])
    calib_path = Path.home() / '.ros2/easy_handeye2/calibrations/eye_to_hand_calib.calib'
    with open(calib_path) as f:
        data = yaml.safe_load(f)
    t = data['transform']['translation']
    r = data['transform']['rotation']
    T_base_cam = np.eye(4)
    T_base_cam[:3, :3] = quat_to_rotmat([r['x'], r['y'], r['z'], r['w']])
    T_base_cam[:3, 3] = [t['x'], t['y'], t['z']]
    T_cam_base = np.linalg.inv(T_base_cam)
    p_cam = (T_cam_base @ np.append(point_base, 1))[:3]
    print(f"输入点(base下): [{point_base[0]:.3f}, {point_base[1]:.3f}, {point_base[2]:.3f}]")
    print(f"变换到相机下: [{p_cam[0]:.3f}, {p_cam[1]:.3f}, {p_cam[2]:.3f}]")
    if p_cam[2] <= 0:
        print("Z<=0, 点在相机后面！标定错了")
        return
    u = FX * p_cam[0] / p_cam[2] + CX
    v = FY * p_cam[1] / p_cam[2] + CY
    print(f"\n投影像素: u={u:.1f}, v={v:.1f}")
    print(f"画面尺寸: {WIDTH}x{HEIGHT}")
    if 0 <= u <= WIDTH and 0 <= v <= HEIGHT:
        print(f"在画面内: 横向{u/WIDTH*100:.0f}%, 纵向{v/HEIGHT*100:.0f}%")
    else:
        print("点在画面外！")
    print(f"\n→ 在相机画面里确认物体实际位置是否接近像素 ({u:.0f}, {v:.0f})")

if __name__ == '__main__':
    main()

