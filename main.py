import sensor, image, time, math, os, tf
from machine import UART
from pyb import LED
uart = UART(12, baudrate=115200)
green_light = LED(2)
blue_light = LED(3)
CMD_MAP	 = 0x53
CMD_DIGIT   = 0x74
CMD_BOX	 = 0x62
CMD_LOCATE  = 0x72
MAP_BRIGHTNESS	 = 10
MAP_AUTO_EXPOSURE  = False
MAP_EXPOSURE_US	= 270
MAP_AUTO_GAIN	  = False
MAP_GAIN_DB		= 0
MAP_AUTO_WHITEBAL  = False
MAP_REGION =(208, 114, 217, 280)
MAP_ROWS = 16
MAP_COLS = 12
MAP_CROP_BORDER = 2
class_avg_colors = {
	0: (45, 49, -83),
	1: (58, 14, -30),
	2: (85, -13, 63),
	3: (60, 88, -56),
	4: (82, -51, 29),
	5: (54, 75, 8)
}
class_std_colors = {
	0: (7, 13, 15),
	1: (25, 18, 21),
	2: (18, 22, 43),
	3: (6, 14, 16),
	4: (19, 21, 39),
	5: (6, 12, 28)
}
DIGIT_ROI_X = 424
DIGIT_ROI_Y = 108
DIGIT_ROI_W = 216
DIGIT_ROI_H = 290
BLUE_SCAN = (64, 97, -64, -18, -30, 8)
GREEN_SCAN = (54, 96, -92, -50, 15, 92)
POS_HISTORY_LEN = 5
SMOOTH_A_NUM = 3
SMOOTH_A_DEN = 10
_loaded_model_type = None
_digit_net = None
_digit_labels = None
_box_net = None
_box_labels = None
_pic_index = 0
cx_smooth, cy_smooth = 0, 0    # 位置平滑上次值
def _init_camera():
	sensor.reset()
	sensor.set_pixformat(sensor.RGB565)
	sensor.set_framesize(sensor.VGA)
	sensor.set_brightness(MAP_BRIGHTNESS)
	sensor.set_auto_exposure(MAP_AUTO_EXPOSURE, MAP_EXPOSURE_US)
	sensor.set_auto_gain(MAP_AUTO_GAIN, MAP_GAIN_DB)
	sensor.set_auto_whitebal(False, rgb_gain_db=(0.0, 0.0, 0.0))
	sensor.skip_frames(time=2000)
def _ensure_model(model_type):
	global _loaded_model_type, _digit_net, _digit_labels, _box_net, _box_labels
	if model_type == 'digit' and _digit_net is not None:
		_loaded_model_type = model_type
		return
	if model_type == 'box' and _box_net is not None:
		_loaded_model_type = model_type
		return
	if model_type == 'digit':
		_digit_labels = [line.rstrip() for line in open("/sd/digits.txt")]
		_digit_net = tf.load("firsteditiondigit.tflite")
		print("Digit model loaded")
	elif model_type == 'box':
		_box_labels = [line.rstrip() for line in open("/sd/classes.txt")]
		_box_net = tf.load("secondedition.tflite")
		print("Box model loaded")
	_loaded_model_type = model_type
def split_image(image, rows, cols):
	height = image.height()
	width = image.width()
	roi_list = []
	for i in range(rows):
		for j in range(cols):
			y_start = int(i * height / rows)
			y_end = int((i + 1) * height / rows)
			x_start = int(j * width / cols)
			x_end = int((j + 1) * width / cols)
			if i == rows - 1:
				y_end = height
			if j == cols - 1:
				x_end = width
			roi_list.append((x_start, y_start, x_end - x_start, y_end - y_start))
	return roi_list
def classify_grid(image, roi):
	x, y, w, h = roi
	if MAP_CROP_BORDER > 0:
		if h > 2 * MAP_CROP_BORDER and w > 2 * MAP_CROP_BORDER:
			x = x + MAP_CROP_BORDER
			y = y + MAP_CROP_BORDER
			w = w - 2 * MAP_CROP_BORDER
			h = h - 2 * MAP_CROP_BORDER
	stats = image.get_statistics(roi=(x, y, w, h))
	grid_l = stats.l_mean()
	grid_a = stats.a_mean()
	grid_b = stats.b_mean()
	grid_l_std = stats.l_stdev()
	grid_a_std = stats.a_stdev()
	grid_b_std = stats.b_stdev()
	min_dist = float('inf')
	best_class = 0
	for class_id in sorted(class_avg_colors.keys()):
		ref_avg = class_avg_colors[class_id]
		ref_std = class_std_colors[class_id]
		avg_dist = math.sqrt((grid_l - ref_avg[0])**2 +
							(grid_a - ref_avg[1])**2 +
							(grid_b - ref_avg[2])**2)
		std_dist = math.sqrt((grid_l_std - ref_std[0])**2 +
							(grid_a_std - ref_std[1])**2 +
							(grid_b_std - ref_std[2])**2)
		total_dist = avg_dist + 0.5 * std_dist
		if total_dist < min_dist:
			min_dist = total_dist
			best_class = class_id
	return best_class
def pack_result(result_array):
	data = bytearray()
	for col in range(MAP_COLS - 1, -1, -1):
		for row in range(MAP_ROWS):
			data.append(result_array[row][col] & 0xFF)
	return data
def recognize_map():
	blue_light.on()
	clock = time.clock()
	clock.tick()
	img = sensor.snapshot()
	x, y, w, h = MAP_REGION
	map_region = img.crop(roi=(x, y, w, h))
	roi_list = split_image(map_region, MAP_ROWS, MAP_COLS)
	result_array = [[0 for _ in range(MAP_COLS)] for _ in range(MAP_ROWS)]
	for i, roi in enumerate(roi_list):
		row = i // MAP_COLS
		col = i % MAP_COLS
		result_array[row][col] = classify_grid(map_region, roi)
	packet = pack_result(result_array)
	green_light.toggle()
	blue_light.off()
	print("Map recognized, %d bytes" % len(packet))
	return packet
def _run_classification(net, labels):
	img = sensor.snapshot()
	img.draw_rectangle(
		(DIGIT_ROI_X, DIGIT_ROI_Y, DIGIT_ROI_W, DIGIT_ROI_H),
		color=(255, 0, 0), thickness=2
	)
	img1 = img.copy(roi=(DIGIT_ROI_X, DIGIT_ROI_Y, DIGIT_ROI_W, DIGIT_ROI_H))
	for obj in tf.classify(net, img1, min_scale=1.0, scale_mul=0.5,
						   x_overlap=0.0, y_overlap=0.0):
		sorted_list = sorted(zip(labels, obj.output()),
							 key=lambda x: x[1], reverse=True)
		top_label = sorted_list[0][0]
		top_conf  = sorted_list[0][1]
		return top_label, top_conf
	return "?", 0.0
def _classify_vote(net, labels, n_frames=3):
	votes = []
	for _ in range(n_frames):
		label, conf = _run_classification(net, labels)
		try:
			idx = labels.index(label)
		except ValueError:
			idx = 0
		votes.append((idx, conf))
	freq = {}
	conf_sum = {}
	for idx, conf in votes:
		freq[idx] = freq.get(idx, 0) + 1
		conf_sum[idx] = conf_sum.get(idx, 0) + conf
	max_freq = max(freq.values())
	candidates = [idx for idx, f in freq.items() if f == max_freq]
	if len(candidates) == 1:
		return candidates[0]
	return max(candidates, key=lambda idx: conf_sum[idx])
def _save_debug_image():
	global _pic_index
	img = sensor.snapshot()
	img.draw_rectangle(
		(DIGIT_ROI_X, DIGIT_ROI_Y, DIGIT_ROI_W, DIGIT_ROI_H),
		color=(255, 0, 0), thickness=2
	)
	roi_img = img.copy(roi=(DIGIT_ROI_X, DIGIT_ROI_Y, DIGIT_ROI_W, DIGIT_ROI_H))
	save_file = "/sd/pic_%d.jpg" % _pic_index
	roi_img.save(save_file)
	print("Saved image:", save_file)
	_pic_index += 1
def _log_result(prefix, idx):
	with open("/sd/result.txt", "a") as f:
		f.write("{} {}\n".format(prefix, idx))
def recognize_digit():
	_ensure_model('digit')
	time.sleep_ms(200)
	blue_light.on()
	idx = _classify_vote(_digit_net, _digit_labels)
	print("Digit voted: idx=%d" % idx)
	_log_result("digit", idx)
	_save_debug_image()
	blue_light.off()
	return bytes([idx])
def recognize_box():
	_ensure_model('box')
	time.sleep_ms(200)
	blue_light.on()
	idx = _classify_vote(_box_net, _box_labels)
	print("Box voted: idx=%d" % idx)
	_log_result("box", idx)
	_save_debug_image()
	blue_light.off()
	return bytes([idx])
def find_target_position(img):
	blobs = img.find_blobs([BLUE_SCAN, GREEN_SCAN],
						   pixels_threshold=80, area_threshold=80,
						   merge=True)
	if not blobs:
		return None
	blobs = sorted(blobs, key=lambda b: b.area(), reverse=True)
	for b in blobs:
		area = b.area()
		w, h = b.w(), b.h()
		avg_side = (w + h) / 2.0
		aspect = w / h if h > 0 else 99
		if not (120 < area < 900):
			continue
		if not (12 < avg_side < 30):
			continue
		if not (0.4 < aspect < 2.5):
			continue
		bx, by, bw, bh = b.rect()
		margin = 5
		roi = (max(0, bx - margin), max(0, by - margin),
			   min(img.width() - bx + margin, bw + 2 * margin),
			   min(img.height() - by + margin, bh + 2 * margin))
		has_blue = img.find_blobs([BLUE_SCAN], roi=roi,
								   pixels_threshold=10, area_threshold=10)
		has_green = img.find_blobs([GREEN_SCAN], roi=roi,
									pixels_threshold=10, area_threshold=10)
		if not has_blue or not has_green:
			continue
		best_blue = max(has_blue, key=lambda x: x.area())
		best_green = max(has_green, key=lambda x: x.area())
		dist = math.sqrt((best_blue.cx() - best_green.cx()) ** 2 +
						 (best_blue.cy() - best_green.cy()) ** 2)
		if dist > 30:
			continue
		img.draw_rectangle(b.rect(), color=(255,255,0))
		img.draw_cross(b.cx(), b.cy(), color=(255,0,0), size=10)
		img.draw_string(b.cx() + 10, b.cy() - 10,
						"({},{})".format(b.cx(), b.cy()), color=(255,255,255))
		return (b.cx(), b.cy())
	return None
def locate_car():
	blue_light.on()
	clock = time.clock()
	clock.tick()
	# 连续拍多帧，取中值 + 低通
	cx_buffer = []
	cy_buffer = []
	for _ in range(POS_HISTORY_LEN):
		img = sensor.snapshot()
		result = find_target_position(img)
		if result is not None:
			cx, cy = result
			cx_buffer.append(cx)
			cy_buffer.append(cy)
	if cx_buffer:
		sorted_cx = sorted(cx_buffer)
		sorted_cy = sorted(cy_buffer)
		cx_med = sorted_cx[len(sorted_cx) // 2]
		cy_med = sorted_cy[len(sorted_cy) // 2]
		# 低通滤波
		global cx_smooth, cy_smooth
		if cx_smooth == 0 and cy_smooth == 0:
			cx_smooth, cy_smooth = cx_med, cy_med
		else:
			cx_smooth = (SMOOTH_A_NUM * cx_med +
						 (SMOOTH_A_DEN - SMOOTH_A_NUM) * cx_smooth) // SMOOTH_A_DEN
			cy_smooth = (SMOOTH_A_NUM * cy_med +
						 (SMOOTH_A_DEN - SMOOTH_A_NUM) * cy_smooth) // SMOOTH_A_DEN
		map_x, map_y, map_w, map_h = MAP_REGION
		adj_x = (map_x + map_w) - cx_smooth  # 距右边界
		adj_y = cy_smooth - map_y
		real_x = int(round(adj_x * MAP_COLS * 20.0 / map_w))
		real_y = int(round(adj_y * MAP_ROWS * 20.0 / map_h))
		msg = "A0X{:d}Y{:d}".format(real_x, real_y)
		print("Located: %s  (pixel=%d,%d)" % (msg, adj_x, adj_y))
		green_light.toggle()
		blue_light.off()
		return bytes(msg, 'utf-8')
	else:
		msg = "A0X0Y0"
		print("Located: Not Found (A0X0Y0)")
		blue_light.off()
		return bytes(msg, 'utf-8')
print("Unified OpenART Ready.")
_init_camera()
print("Camera initialized (VGA, fixed exposure/gain/whitebal)")
print("  'S' → Map Recognition")
print("  't' → Digit Recognition")
print("  'b' → Box/Image Recognition")
print("  'r' → Car Localization")
while True:
	if uart.any():
		cmd = uart.read(1)
		if cmd:
			cmd_byte = cmd[0]
			if cmd_byte == CMD_MAP:
				response = recognize_map()
				uart.write(response)
			elif cmd_byte == CMD_DIGIT:
				response = recognize_digit()
				uart.write(response)
			elif cmd_byte == CMD_BOX:
				response = recognize_box()
				uart.write(response)
			elif cmd_byte == CMD_LOCATE:
				response = locate_car()
				uart.write(response)
			else:
				uart.write(cmd)