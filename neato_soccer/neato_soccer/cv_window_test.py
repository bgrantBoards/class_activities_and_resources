import cv2

def set_red_lower_bound(val):
    print(f"set: {val}")

cv2.namedWindow('video_window')
cv2.namedWindow('binary_window')
cv2.namedWindow('image_info')
# self.red_lower_bound = 0
cv2.createTrackbar('red lower bound', 'binary_window', 0, 255, set_red_lower_bound)

while True:
    cv2.waitKey(5)