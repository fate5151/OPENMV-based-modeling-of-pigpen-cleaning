import cv2 #opencv读取的格式是BGR
import matplotlib.pyplot as plt
import numpy as np 

def cv_show(name,img):
    cv2.imshow(name,img) 
    cv2.waitKey(0) 
    cv2.destroyAllWindows()
# 初始化摄像头
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
# 存储背景图像
cv2.imwrite('background.png', frame)
cap.release()

cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if ret:
        # 显示图像
        cv2.imshow('Live Feed', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()

gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 转换为灰度图
ret, binary_pig = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)  # Otsu分割

background = cv2.imread('background.png', cv2.IMREAD_GRAYSCALE)  # 读取背景图像
diff = cv2.absdiff(gray, background)  # 图像差分
ret, binary_diff = cv2.threshold(diff, 50, 255, cv2.THRESH_BINARY)  # 二值化

binary_feces = cv2.absdiff(binary_diff, binary_pig)  # 提取粪尿部分

white_pixels = cv2.countNonZero(binary_feces)  # 计算白色像素点数量
total_pixels = binary_feces.size  # 图像总像素点数
ratio = white_pixels / total_pixels  # 计算白色像素占比

T0 = 0.05  # 设置阈值
if ratio < T0:
    print("猪舍洁净")
else:
    print("猪舍需要清扫")

cv_show('background',background)
cv_show('gray',gray)
cv_show('binary_pig',binary_pig)