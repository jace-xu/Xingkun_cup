# -*- coding: utf-8 -*-
"""
树莓派轻量视觉瞄准 + 透视圆轨迹一体化主程序（USB 摄像头）

功能概述：
1) 默认运行 Task1：矩形目标中心追踪，输出目标中心与画面中心的像素误差。
2) 串口收到“切换指令”后切换到 Task2：透视校正下的圆形轨迹目标点输出。
3) 串口收到“切换指令”后可再切回 Task1。
4) 两个任务都保持与原脚本一致的可视化显示与调试窗口。

注意：
- 本文件集成了 task_1.py 和 task_2.py 的主要逻辑。
- 注释已尽量写得详细，便于理解每一步在做什么。
"""

import math
import struct
import time

import cv2
import numpy as np

try:
	import serial
except Exception:
	serial = None

try:
	import RPi.GPIO as GPIO
except Exception:
	GPIO = None


# =========================
# 配置区（相机与处理参数）
# =========================
# 摄像头基础参数
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
TARGET_FPS = 60

# 预处理参数（轻量）
GAUSSIAN_KSIZE = 5
BINARY_THRESHOLD = 70
CANNY_LOW = 50
CANNY_HIGH = 150
MORPH_KERNEL = 3

# 可选：使用自适应阈值（鲁棒性更好，但略增计算量）
USE_ADAPTIVE_THRESHOLD = False
ADAPTIVE_BLOCK_SIZE = 31
ADAPTIVE_C = 8

# 矩形筛选参数
ANGLE_TOLERANCE = 30
SIDE_RATIO_TOLERANCE = 0.4
MIN_CONTOUR_AREA = 1000
MAX_CONTOUR_AREA = 307200
MIN_PERIMETER = 20
APPROX_EPSILON_FACTOR = 0.02
NESTED_AREA_RATIO_MIN = 0.7
BBOX_CONTAIN_TOLERANCE = 3

# 误差平滑参数（指数滑动平均）
ENABLE_ERROR_EMA = True
ERROR_EMA_ALPHA = 0.35

# 串口参数（按下位机修改）
SERIAL_ENABLE = True
SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200
MAX_ERROR_ABS = 5000

# =========================
# GPIO 任务切换配置
# =========================
# 外部开关公共端接 GND，因此输入需要上拉，低电平有效
GPIO_TASK1_PIN = 23
GPIO_TASK2_PIN = 24
GPIO_TASK1 = 1
GPIO_TASK2 = 2
GPIO_OUT_PIN = 25

# 任务切换后 GPIO25 延迟拉高
GPIO_OUT_DELAY_SEC = 3.5

# 屏幕中心偏移（向左上）
CAM_CENTER_OFFSET_X = 24
CAM_CENTER_OFFSET_Y = 20

# =========================
# 透视与圆轨迹参数（Task2）
# =========================
# 目标矩形的实际尺寸（毫米），默认 A4
TARGET_WIDTH_MM = 297
TARGET_HEIGHT_MM = 210

# 透视校正后的像素密度（像素/毫米）
PIXELS_PER_MM = 2.0

# 目标圆的物理直径（毫米）
CIRCLE_DIAMETER_MM = 120

# 转一圈的时间（秒）
CYCLE_SECONDS = 30.0

# 是否只转一圈后停留（False 则持续循环）
STOP_AFTER_ONE_CYCLE = False

# 轨迹采样点数量（用于在原图绘制圆轨迹）
CIRCLE_SAMPLE_POINTS = 120

# 窗口名称（保持与原脚本一致）
WINDOW_TASK1 = "Pi Vision"
WINDOW_TASK2 = "Pi Vision Task2"
WINDOW_BINARY = "binary"
WINDOW_EDGES = "edges"
WINDOW_CORRECTED = "corrected"


class Task1State:
	"""保存 Task1 的运行状态（主要是平滑历史值）。"""

	def __init__(self):
		self.prev_dx = None
		self.prev_dy = None

	def reset_smoothing(self):
		"""切换任务时清空平滑历史，避免突变影响输出。"""
		self.prev_dx = None
		self.prev_dy = None


class Task2State:
	"""保存 Task2 的运行状态（平滑 + 圆周运动时间）。"""

	def __init__(self):
		self.prev_dx = None
		self.prev_dy = None
		self.cycle_start = time.time()
		self.cycle_done = False
		self.gpio_ready = False

	def reset_smoothing(self):
		"""切换任务时清空平滑历史，避免突变影响输出。"""
		self.prev_dx = None
		self.prev_dy = None

	def reset_cycle(self):
		"""切换到 Task2 时重置圆周运动起点。"""
		self.cycle_start = time.time()
		self.cycle_done = False
		self.gpio_ready = False


def pack_frame(cmd_id, flags, dx, dy):
	"""
	将误差数据打包为二进制帧（用于发送到下位机）。

	协议格式:
	[0xA5][dx(i16)][dy(i16)]
	"""
	return struct.pack("<Bhh", 0xA5, int(dx), int(dy))


def init_serial():
	"""
	初始化串口对象。

	当 SERIAL_ENABLE=False 或 pyserial 不可用时，返回 None。
	"""
	if not SERIAL_ENABLE:
		return None

	if serial is None:
		print("[WARN] 未安装 pyserial，串口发送/接收已禁用")
		return None

	try:
		# timeout=0 表示非阻塞读取，便于主循环实时性
		ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0)
		print(f"[OK] 串口已连接: {SERIAL_PORT} @ {SERIAL_BAUD}")
		return ser
	except Exception as exc:
		print(f"[WARN] 串口打开失败: {exc}")
		return None


def send_error(ser, dx, dy):
	"""
	发送误差数据（像素单位），异常时不阻塞主循环。
	"""
	if ser is None:
		return

	try:
		dx = int(round(dx))
		dy = int(round(dy))
		dx = max(-MAX_ERROR_ABS, min(MAX_ERROR_ABS, dx))
		dy = max(-MAX_ERROR_ABS, min(MAX_ERROR_ABS, dy))
		frame = pack_frame(0x0200, 0x0001, dx, dy)
		ser.write(frame)
	except Exception:
		pass


def poll_gpio_task():
	"""
	轮询 GPIO 引脚电平，决定任务切换。

	Returns:
		int | None: 1 表示切换到 Task1，2 表示切换到 Task2，无变化返回 None。
	"""
	if GPIO is None:
		return None

	state_17 = GPIO.input(GPIO_TASK1_PIN)
	state_27 = GPIO.input(GPIO_TASK2_PIN)

	if state_17 == GPIO.LOW and state_27 == GPIO.HIGH:
		return GPIO_TASK1
	if state_27 == GPIO.LOW and state_17 == GPIO.HIGH:
		return GPIO_TASK2
	return None


def calculate_angle(p1, p2, p3):
	"""计算三点构成的角度（以 p2 为顶点）。"""
	v1 = np.array([p1[0] - p2[0], p1[1] - p2[1]], dtype=np.float32)
	v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]], dtype=np.float32)
	len1 = np.linalg.norm(v1)
	len2 = np.linalg.norm(v2)
	if len1 < 1e-6 or len2 < 1e-6:
		return 0.0
	cos_angle = float(np.dot(v1, v2) / (len1 * len2))
	cos_angle = float(np.clip(cos_angle, -1.0, 1.0))
	return float(np.degrees(np.arccos(cos_angle)))


def calculate_side_lengths(corners):
	"""计算四边形的四条边长。"""
	pts = corners.reshape(4, 2).astype(np.float32)
	sides = []
	for i in range(4):
		p1 = pts[i]
		p2 = pts[(i + 1) % 4]
		sides.append(float(np.linalg.norm(p2 - p1)))
	return sides


def check_rectangle_geometry(corners, angle_tolerance=None, side_ratio_tolerance=None):
	"""按角度与对边长度比例检查四边形几何合理性。"""
	if angle_tolerance is None:
		angle_tolerance = ANGLE_TOLERANCE
	if side_ratio_tolerance is None:
		side_ratio_tolerance = SIDE_RATIO_TOLERANCE

	pts = corners.reshape(4, 2).astype(np.float32)
	angles = []
	for i in range(4):
		p_prev = pts[(i - 1) % 4]
		p_curr = pts[i]
		p_next = pts[(i + 1) % 4]
		angles.append(calculate_angle(p_prev, p_curr, p_next))

	angle_check = all(abs(angle - 90.0) <= angle_tolerance for angle in angles)

	sides = calculate_side_lengths(pts)
	max_side_0 = max(sides[0], sides[2])
	max_side_1 = max(sides[1], sides[3])
	side1_ratio = abs(sides[0] - sides[2]) / max_side_0 if max_side_0 > 0 else 1.0
	side2_ratio = abs(sides[1] - sides[3]) / max_side_1 if max_side_1 > 0 else 1.0
	side_check = side1_ratio <= side_ratio_tolerance and side2_ratio <= side_ratio_tolerance

	min_side = min(sides)
	max_side = max(sides)
	side_aspect_ratio = max_side / min_side if min_side > 0 else float("inf")
	reasonable_shape = side_aspect_ratio <= 10 and min_side >= 20

	return angle_check and side_check and reasonable_shape


def is_convex_quad(corners):
	"""判断四边形是否为凸四边形。"""
	try:
		return bool(cv2.isContourConvex(corners))
	except Exception:
		return False


def is_rectangle(corners):
	"""检查四边形是否满足矩形几何约束。"""
	if not is_convex_quad(corners):
		return False
	return check_rectangle_geometry(corners)


def corners_to_bbox(corners):
	"""由四边形角点计算外接包围盒。"""
	pts = corners.reshape(4, 2).astype(np.float32)
	min_x = float(np.min(pts[:, 0]))
	min_y = float(np.min(pts[:, 1]))
	max_x = float(np.max(pts[:, 0]))
	max_y = float(np.max(pts[:, 1]))
	return min_x, min_y, max_x, max_y


def corners_inside_bbox(corners, bbox, tolerance=0.0):
	"""判断角点是否基本位于目标包围盒内部。"""
	min_x, min_y, max_x, max_y = bbox
	pts = corners.reshape(4, 2).astype(np.float32)
	x_ok = np.logical_and(pts[:, 0] >= min_x - tolerance, pts[:, 0] <= max_x + tolerance)
	y_ok = np.logical_and(pts[:, 1] >= min_y - tolerance, pts[:, 1] <= max_y + tolerance)
	return bool(np.all(x_ok) and np.all(y_ok))


def diagonal_intersection(corners):
	"""通过四边形两条对角线的交点计算中心。"""
	pts = corners.reshape(4, 2).astype(np.float32)
	p0, p1, p2, p3 = pts[0], pts[1], pts[2], pts[3]
	x1, y1 = p0
	x2, y2 = p2
	x3, y3 = p1
	x4, y4 = p3

	denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
	if abs(denom) < 1e-6:
		return None

	px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
	py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
	return int(round(px)), int(round(py))


def preprocess(frame):
	"""
	对输入图像执行轻量预处理。

	返回: combined（二值+边缘融合）、binary（二值）、edges（边缘）。
	"""
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	blur = cv2.GaussianBlur(gray, (GAUSSIAN_KSIZE, GAUSSIAN_KSIZE), 0)

	if USE_ADAPTIVE_THRESHOLD:
		binary = cv2.adaptiveThreshold(
			blur,
			255,
			cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
			cv2.THRESH_BINARY_INV,
			ADAPTIVE_BLOCK_SIZE,
			ADAPTIVE_C,
		)
	else:
		_, binary = cv2.threshold(blur, BINARY_THRESHOLD, 255, cv2.THRESH_BINARY_INV)

	kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (MORPH_KERNEL, MORPH_KERNEL))
	binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

	edges = cv2.Canny(blur, CANNY_LOW, CANNY_HIGH)
	combined = cv2.bitwise_or(binary, edges)
	return combined, binary, edges


def detect_best_rectangle(mask, frame_shape):
	"""
	在掩码中检测最佳矩形，并优先匹配内外嵌套矩形对。
	"""
	contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	if not contours:
		return None, None, 0, None

	height, width = frame_shape[:2]
	max_area = min(float(height * width), float(MAX_CONTOUR_AREA))

	best_strict = None
	best_strict_area = 0.0
	best_any = None
	best_any_area = 0.0

	candidates = []
	candidate_map = {}

	for idx, contour in enumerate(contours):
		area = cv2.contourArea(contour)
		if area < MIN_CONTOUR_AREA or area > max_area:
			continue

		perimeter = cv2.arcLength(contour, True)
		if perimeter < MIN_PERIMETER:
			continue

		approx = cv2.approxPolyDP(contour, APPROX_EPSILON_FACTOR * perimeter, True)
		if len(approx) != 4:
			continue
		if not is_convex_quad(approx):
			continue

		center = diagonal_intersection(approx)
		if center is None:
			continue
		cx, cy = center

		if area > best_any_area:
			best_any_area = area
			best_any = (approx.reshape(-1, 1, 2), (cx, cy), area)

		if is_rectangle(approx):
			if area > best_strict_area:
				best_strict_area = area
				best_strict = (approx.reshape(-1, 1, 2), (cx, cy), area)
			corners = approx.reshape(-1, 1, 2)
			candidate = {
				"index": idx,
				"corners": corners,
				"center": (cx, cy),
				"area": area,
				"bbox": corners_to_bbox(corners),
			}
			candidates.append(candidate)
			candidate_map[idx] = candidate

	# ---------- 优先级 1：hierarchy 父子关系 ----------
	if hierarchy is not None and candidates:
		h = hierarchy[0]
		best_pair = None
		best_outer_area = 0.0
		best_ratio = 0.0
		for inner in candidates:
			parent_idx = int(h[inner["index"]][3])
			if parent_idx < 0:
				continue
			outer = candidate_map.get(parent_idx)
			if outer is None:
				continue
			if inner["area"] >= outer["area"]:
				continue
			ratio = inner["area"] / outer["area"]
			if ratio < NESTED_AREA_RATIO_MIN:
				continue
			if outer["area"] > best_outer_area or (
				outer["area"] == best_outer_area and ratio > best_ratio
			):
				best_outer_area = outer["area"]
				best_ratio = ratio
				best_pair = (outer, inner)

		if best_pair is not None:
			outer = best_pair[0]
			return outer["corners"], outer["center"], outer["area"], 1

	# ---------- 优先级 2：包围盒包含关系 ----------
	if len(candidates) >= 2:
		best_pair = None
		best_outer_area = 0.0
		best_ratio = 0.0
		for outer in candidates:
			outer_bbox = outer["bbox"]
			for inner in candidates:
				if inner is outer:
					continue
				if inner["area"] >= outer["area"]:
					continue
				ratio = inner["area"] / outer["area"]
				if ratio < NESTED_AREA_RATIO_MIN:
					continue
				if not corners_inside_bbox(inner["corners"], outer_bbox, BBOX_CONTAIN_TOLERANCE):
					continue
				if outer["area"] > best_outer_area or (
					outer["area"] == best_outer_area and ratio > best_ratio
				):
					best_outer_area = outer["area"]
					best_ratio = ratio
					best_pair = (outer, inner)

		if best_pair is not None:
			outer = best_pair[0]
			return outer["corners"], outer["center"], outer["area"], 2

	# ---------- 优先级 3：回退选择 ----------
	if best_strict is not None:
		return best_strict[0], best_strict[1], best_strict[2], 3
	if best_any is not None:
		return best_any[0], best_any[1], best_any[2], 3
	return None, None, 0, None


def apply_error_smoothing(raw_dx, raw_dy, prev_dx, prev_dy):
	"""对误差执行指数滑动平均（EMA）平滑。"""
	if (not ENABLE_ERROR_EMA) or prev_dx is None or prev_dy is None:
		return int(raw_dx), int(raw_dy), float(raw_dx), float(raw_dy)

	alpha = ERROR_EMA_ALPHA
	smooth_dx = alpha * raw_dx + (1.0 - alpha) * prev_dx
	smooth_dy = alpha * raw_dy + (1.0 - alpha) * prev_dy
	return int(round(smooth_dx)), int(round(smooth_dy)), smooth_dx, smooth_dy


def draw_task1_overlay(
	frame,
	cam_center,
	rect_corners,
	rect_center,
	dx,
	dy,
	fps,
	select_level,
	gpio_state,
):
	"""绘制 Task1 的可视化叠加信息。"""
	cv2.circle(frame, cam_center, 6, (255, 255, 0), -1)
	cv2.putText(
		frame,
		f"CamCenter: {cam_center}",
		(10, 25),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.55,
		(255, 255, 0),
		2,
	)

	if rect_corners is not None and rect_center is not None:
		cv2.drawContours(frame, [rect_corners], -1, (0, 255, 0), 2)
		cv2.circle(frame, rect_center, 6, (0, 0, 255), -1)
		cv2.line(frame, cam_center, rect_center, (0, 255, 255), 2)
		cv2.putText(
			frame,
			f"TargetCenter: {rect_center}",
			(10, 50),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.55,
			(0, 0, 255),
			2,
		)
		cv2.putText(
			frame,
			f"Error dx: {dx:+d}px  dy: {dy:+d}px",
			(10, 75),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.65,
			(0, 255, 255),
			2,
		)
	else:
		cv2.putText(
			frame,
			"Target: Not Found",
			(10, 50),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.65,
			(0, 0, 255),
			2,
		)

	cv2.putText(
		frame,
		f"FPS: {fps:.1f}",
		(10, 100),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.55,
		(255, 255, 255),
		1,
	)
	if select_level is not None:
		cv2.putText(
			frame,
			f"Select: L{select_level}",
			(10, 125),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.55,
			(200, 255, 200),
			1,
		)
	cv2.putText(
		frame,
		"Mode: Task1",
		(10, 150),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.55,
		(200, 200, 255),
		1,
	)
	gpio_text = "HIGH" if gpio_state.get("is_high") else "LOW"
	cv2.putText(
		frame,
		f"GPIO25: {gpio_text}",
		(10, 175),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.55,
		(255, 255, 255),
		1,
	)
	cv2.putText(
		frame,
		"Q:Quit  B:Binary  E:Edges",
		(10, frame.shape[0] - 15),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.5,
		(200, 200, 200),
		1,
	)


def draw_task2_overlay(
	frame,
	cam_center,
	rect_corners,
	target_point,
	dx,
	dy,
	fps,
	select_level,
	gpio_state,
):
	"""绘制 Task2 的可视化叠加信息。"""
	cv2.circle(frame, cam_center, 6, (255, 255, 0), -1)
	cv2.putText(
		frame,
		f"CamCenter: {cam_center}",
		(10, 25),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.55,
		(255, 255, 0),
		2,
	)

	if rect_corners is not None:
		cv2.drawContours(frame, [rect_corners], -1, (0, 255, 0), 2)

	if target_point is not None:
		cv2.circle(frame, target_point, 6, (0, 0, 255), -1)
		cv2.line(frame, cam_center, target_point, (0, 255, 255), 2)
		cv2.putText(
			frame,
			f"TargetPoint: {target_point}",
			(10, 50),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.55,
			(0, 0, 255),
			2,
		)
		cv2.putText(
			frame,
			f"Error dx: {dx:+d}px  dy: {dy:+d}px",
			(10, 75),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.65,
			(0, 255, 255),
			2,
		)
	else:
		cv2.putText(
			frame,
			"Target: Not Found",
			(10, 50),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.65,
			(0, 0, 255),
			2,
		)

	cv2.putText(
		frame,
		f"FPS: {fps:.1f}",
		(10, 100),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.55,
		(255, 255, 255),
		1,
	)
	if select_level is not None:
		cv2.putText(
			frame,
			f"Select: L{select_level}",
			(10, 125),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.55,
			(200, 255, 200),
			1,
		)
	cv2.putText(
		frame,
		"Mode: Task2",
		(10, 150),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.55,
		(200, 200, 255),
		1,
	)
	gpio_text = "HIGH" if gpio_state.get("is_high") else "LOW"
	cv2.putText(
		frame,
		f"GPIO25: {gpio_text}",
		(10, 175),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.55,
		(255, 255, 255),
		1,
	)
	cv2.putText(
		frame,
		"Q:Quit  B:Binary  E:Edges  C:Corrected",
		(10, frame.shape[0] - 15),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.5,
		(200, 200, 200),
		1,
	)


def safe_close_window(window_name):
	"""安全关闭窗口，避免未创建窗口导致异常。"""
	try:
		visible = cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE)
		if visible >= 0:
			cv2.destroyWindow(window_name)
	except Exception:
		pass


def safe_toggle_window(window_name, show_flag, image):
	"""安全控制调试窗口显示/关闭。"""
	if show_flag:
		cv2.imshow(window_name, image)
		return
	safe_close_window(window_name)


def sort_corners(corners):
	"""对四个角点进行排序：左上、右上、右下、左下。"""
	pts = corners.reshape(4, 2).astype(np.float32)
	sums = pts[:, 0] + pts[:, 1]
	diffs = pts[:, 0] - pts[:, 1]

	top_left = pts[np.argmin(sums)]
	bottom_right = pts[np.argmax(sums)]
	top_right = pts[np.argmax(diffs)]
	bottom_left = pts[np.argmin(diffs)]

	return np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.float32)


def create_perspective_transform(src_corners):
	"""根据矩形角点创建透视变换矩阵，并给出校正图像尺寸。"""
	target_width_px = int(TARGET_WIDTH_MM * PIXELS_PER_MM)
	target_height_px = int(TARGET_HEIGHT_MM * PIXELS_PER_MM)
	corrected_size = (target_width_px, target_height_px)

	dst_corners = np.array(
		[
			[0, 0],
			[target_width_px - 1, 0],
			[target_width_px - 1, target_height_px - 1],
			[0, target_height_px - 1],
		],
		dtype=np.float32,
	)

	src_sorted = sort_corners(src_corners)

	M = cv2.getPerspectiveTransform(src_sorted, dst_corners)
	M_inv = cv2.getPerspectiveTransform(dst_corners, src_sorted)
	return M, M_inv, corrected_size


def transform_point_back(point, M_inv):
	"""将校正图坐标点反投影到原图坐标。"""
	pts = np.array([[point]], dtype=np.float32)
	transformed = cv2.perspectiveTransform(pts, M_inv)
	x, y = transformed[0][0]
	return int(round(x)), int(round(y))


def generate_circle_point(center, radius_px, t_sec, period_sec):
	"""根据时间生成圆周上的目标点（从顶部开始）。"""
	phase = (t_sec / period_sec) % 1.0
	angle = -math.pi / 2 + 2.0 * math.pi * phase

	cx, cy = center
	x = cx + radius_px * math.cos(angle)
	y = cy + radius_px * math.sin(angle)
	return float(x), float(y)


def build_projected_circle_points(center, radius_px, M_inv, samples=CIRCLE_SAMPLE_POINTS):
	"""将校正图中的圆投影回原图，生成折线用于显示轨迹。"""
	pts = []
	for i in range(samples):
		angle = 2.0 * math.pi * (i / float(samples))
		x = center[0] + radius_px * math.cos(angle)
		y = center[1] + radius_px * math.sin(angle)
		pts.append([x, y])

	pts_np = np.array(pts, dtype=np.float32).reshape(-1, 1, 2)
	projected = cv2.perspectiveTransform(pts_np, M_inv)
	return projected.astype(np.int32)


def draw_corrected_view(corrected_img, circle_center, radius_px, target_pt):
	"""在校正视图上绘制圆形轨迹与目标点。"""
	canvas = corrected_img.copy()
	cv2.circle(canvas, circle_center, radius_px, (255, 0, 255), 2)
	cv2.circle(canvas, circle_center, 4, (255, 255, 0), -1)
	if target_pt is not None:
		cv2.circle(canvas, target_pt, 6, (0, 0, 255), -1)

	cv2.putText(
		canvas,
		"Corrected View + Circle Target",
		(10, 25),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.6,
		(255, 255, 255),
		2,
	)
	return canvas


def task1_process(frame, cam_center, state):
	"""执行 Task1 核心逻辑，返回检测结果与误差。"""
	combined, binary, edges = preprocess(frame)
	rect_corners, rect_center, _area, select_level = detect_best_rectangle(combined, frame.shape)

	if rect_center is not None:
		raw_dx = rect_center[0] - cam_center[0]
		raw_dy = cam_center[1] - rect_center[1]
	else:
		raw_dx = 0
		raw_dy = 0

	dx, dy, smooth_dx, smooth_dy = apply_error_smoothing(raw_dx, raw_dy, state.prev_dx, state.prev_dy)
	state.prev_dx, state.prev_dy = smooth_dx, smooth_dy

	return {
		"binary": binary,
		"edges": edges,
		"rect_corners": rect_corners,
		"rect_center": rect_center,
		"dx": dx,
		"dy": dy,
		"select_level": select_level,
	}


def task2_process(frame, cam_center, state, show_corrected, gpio_state):
	"""执行 Task2 核心逻辑，返回目标点、误差与可视化数据。"""
	combined, binary, edges = preprocess(frame)
	rect_corners, _rect_center, _area, select_level = detect_best_rectangle(combined, frame.shape)

	target_point = None
	corrected_view = None
	projected_circle = None

	if rect_corners is not None:
		# 计算透视变换矩阵（不一定显示校正图，但必须有 M/M_inv）
		M, M_inv, corrected_size = create_perspective_transform(rect_corners)

		# 计算圆心与半径（像素）
		circle_center = (corrected_size[0] // 2, corrected_size[1] // 2)
		radius_px = int((CIRCLE_DIAMETER_MM * 0.5) * PIXELS_PER_MM)

		# 根据时间生成圆周目标点（GPIO25 拉高后才开始运动）
		gpio_high = bool(gpio_state.get("is_high")) if gpio_state is not None else False
		if not gpio_high:
			state.gpio_ready = False
			elapsed = 0.0
		else:
			if not state.gpio_ready:
				state.gpio_ready = True
				state.cycle_start = time.time()
				state.cycle_done = False
			elapsed = time.time() - state.cycle_start
			if STOP_AFTER_ONE_CYCLE and elapsed >= CYCLE_SECONDS:
				state.cycle_done = True
			if state.cycle_done:
				elapsed = CYCLE_SECONDS
		target_x, target_y = generate_circle_point(circle_center, radius_px, elapsed, CYCLE_SECONDS)

		# 反投影到原图坐标
		target_point = transform_point_back((target_x, target_y), M_inv)

		# 生成轨迹投影点用于绘制
		projected_circle = build_projected_circle_points(circle_center, radius_px, M_inv)

		# 只有在需要显示校正图时才生成校正图像，节省计算
		if show_corrected:
			corrected = cv2.warpPerspective(frame, M, corrected_size)
			corrected_view = draw_corrected_view(
				corrected,
				circle_center,
				radius_px,
				(int(round(target_x)), int(round(target_y))),
			)

	# 误差计算
	if target_point is not None:
		raw_dx = target_point[0] - cam_center[0]
		raw_dy = cam_center[1] - target_point[1]
	else:
		raw_dx = 0
		raw_dy = 0

	dx, dy, smooth_dx, smooth_dy = apply_error_smoothing(raw_dx, raw_dy, state.prev_dx, state.prev_dy)
	state.prev_dx, state.prev_dy = smooth_dx, smooth_dy

	return {
		"binary": binary,
		"edges": edges,
		"rect_corners": rect_corners,
		"target_point": target_point,
		"dx": dx,
		"dy": dy,
		"select_level": select_level,
		"corrected_view": corrected_view,
		"projected_circle": projected_circle,
	}


def switch_task(current_task, new_task, task1_state, task2_state):
	"""
	切换任务时统一重置状态并打印提示。
	"""
	if new_task == current_task:
		return current_task

	if new_task == GPIO_TASK1:
		task1_state.reset_smoothing()
		print("[INFO] 切换到 Task1")
	elif new_task == GPIO_TASK2:
		task2_state.reset_smoothing()
		task2_state.reset_cycle()
		print("[INFO] 切换到 Task2")

	return new_task


def schedule_gpio25_high(gpio_state):
	"""安排 GPIO25 延迟拉高（非阻塞）。"""
	if GPIO is None:
		return

	gpio_state["pending_until"] = time.time() + GPIO_OUT_DELAY_SEC
	gpio_state["is_high"] = False
	GPIO.output(GPIO_OUT_PIN, GPIO.LOW)


def update_gpio25(gpio_state):
	"""检查是否到时拉高 GPIO25。"""
	if GPIO is None:
		return

	pending_until = gpio_state.get("pending_until")
	if pending_until is None:
		return
	if time.time() >= pending_until:
		GPIO.output(GPIO_OUT_PIN, GPIO.HIGH)
		gpio_state["pending_until"] = None
		gpio_state["is_high"] = True


def run():
	"""主运行函数，默认 Task1，串口控制切换。"""
	print("=== Raspberry Pi USB Camera Integrated Tracker ===")

	# ---------- 摄像头初始化 ----------
	cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
	if not cap.isOpened():
		cap = cv2.VideoCapture(CAMERA_INDEX)
	if not cap.isOpened():
		print("[ERR] 无法打开 USB 摄像头")
		return

	cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
	cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
	cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

	# ---------- 串口初始化 ----------
	ser = init_serial()

	# ---------- GPIO 初始化 ----------
	if GPIO is None:
		print("[ERR] 未检测到 RPi.GPIO，请确认已安装 rpi-lgpio 或 RPi.GPIO")
		return
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(GPIO_TASK1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(GPIO_TASK2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.setup(GPIO_OUT_PIN, GPIO.OUT, initial=GPIO.LOW)

	# ---------- 调试显示状态 ----------
	show_binary = False
	show_edges = False
	show_corrected = True

	# ---------- 性能统计 ----------
	frame_count = 0
	t0 = time.time()

	# ---------- 任务状态 ----------
	current_task = GPIO_TASK1
	task1_state = Task1State()
	task2_state = Task2State()
	gpio_state = {"pending_until": None, "is_high": False}

	# ---------- 启动默认任务后的 GPIO25 输出 ----------
	schedule_gpio25_high(gpio_state)

	try:
		while True:
			# 1) 采集一帧
			ok, frame = cap.read()
			if not ok or frame is None:
				continue

			# 2) 摄像头倒装，先旋转 180 度再进入后续流程
			frame = cv2.rotate(frame, cv2.ROTATE_180)

			# 3) 统一分辨率，避免后续处理尺寸抖动
			frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT), interpolation=cv2.INTER_LINEAR)

			# 4) 计算画面中心
			cam_cx, cam_cy = FRAME_WIDTH // 2, FRAME_HEIGHT // 2
			cam_center = (cam_cx - CAM_CENTER_OFFSET_X, cam_cy - CAM_CENTER_OFFSET_Y)


			# 5) 轮询 GPIO 切换命令
			desired_task = poll_gpio_task()
			if desired_task in (GPIO_TASK1, GPIO_TASK2) and desired_task != current_task:
				current_task = switch_task(current_task, desired_task, task1_state, task2_state)
				schedule_gpio25_high(gpio_state)

			# 5.1) 后台更新 GPIO25 延迟拉高
			update_gpio25(gpio_state)

			# 6) 任务分发：根据当前任务执行不同处理流程
			if current_task == GPIO_TASK1:
				result = task1_process(frame, cam_center, task1_state)

				# 6.1) 误差发送
				send_error(ser, result["dx"], result["dy"])

				# 6.2) FPS 统计
				frame_count += 1
				elapsed = time.time() - t0
				fps = frame_count / elapsed if elapsed > 1e-6 else 0.0

				# 6.3) 叠加显示
				draw_task1_overlay(
					frame,
					cam_center,
					result["rect_corners"],
					result["rect_center"],
					result["dx"],
					result["dy"],
					fps,
					result["select_level"],
					gpio_state,
				)

				# 6.4) 显示主窗口
				cv2.imshow(WINDOW_TASK1, frame)
				safe_close_window(WINDOW_TASK2)

				# 6.5) 调试窗口显示
				safe_toggle_window(WINDOW_BINARY, show_binary, result["binary"])
				safe_toggle_window(WINDOW_EDGES, show_edges, result["edges"])
				safe_close_window(WINDOW_CORRECTED)

			else:
				result = task2_process(frame, cam_center, task2_state, show_corrected, gpio_state)

				# 6.1) 误差发送
				send_error(ser, result["dx"], result["dy"])

				# 6.2) FPS 统计
				frame_count += 1
				elapsed = time.time() - t0
				fps = frame_count / elapsed if elapsed > 1e-6 else 0.0

				# 6.3) 轨迹绘制（原图）
				if result["projected_circle"] is not None:
					cv2.polylines(frame, [result["projected_circle"]], True, (255, 0, 255), 2)

				# 6.4) 叠加显示
				draw_task2_overlay(
					frame,
					cam_center,
					result["rect_corners"],
					result["target_point"],
					result["dx"],
					result["dy"],
					fps,
					result["select_level"],
					gpio_state,
				)

				# 6.5) 显示主窗口
				cv2.imshow(WINDOW_TASK2, frame)
				safe_close_window(WINDOW_TASK1)

				# 6.6) 调试窗口显示
				safe_toggle_window(WINDOW_BINARY, show_binary, result["binary"])
				safe_toggle_window(WINDOW_EDGES, show_edges, result["edges"])
				if show_corrected and result["corrected_view"] is not None:
					cv2.imshow(WINDOW_CORRECTED, result["corrected_view"])
				else:
					safe_close_window(WINDOW_CORRECTED)

			# 7) 键盘交互
			key = cv2.waitKey(1) & 0xFF
			if key in (ord("q"), ord("Q")):
				break
			if key in (ord("b"), ord("B")):
				show_binary = not show_binary
			if key in (ord("e"), ord("E")):
				show_edges = not show_edges
			if key in (ord("c"), ord("C")):
				show_corrected = not show_corrected

	finally:
		# ---------- 资源释放 ----------
		if ser is not None:
			try:
				ser.close()
			except Exception:
				pass

		if GPIO is not None:
			try:
				GPIO.cleanup()
			except Exception:
				pass

		cap.release()
		cv2.destroyAllWindows()
		print("[OK] 资源已释放，程序退出")


if __name__ == "__main__":
	run()
