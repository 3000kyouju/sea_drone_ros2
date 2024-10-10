#!/home/nakahira/work/yolov8/bin/python
import cv2
import numpy as np
import rclpy
from .drone_aruco import DroneControl
#import drone
#!/usr/bin/env python3
def main(args=None):
    # ウェブカメラのキャプチャを開始
    video_path = 2
    cap = cv2.VideoCapture(video_path)
    img_size = np.array([cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT)])
    # fps = int(cap.get(cv2.CAP_PROP_FPS))
    # w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) 
    # h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))  
    # fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    # writer = cv2.VideoWriter('20240819.m4v', fourcc, fps, (w, h))  

    dictionaly = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionaly)

    rclpy.init()
    
    drone_obj = DroneControl(img_size) 
    rate = drone_obj.create_rate(10)
    # キャプチャがオープンしている間続ける
    while(cap.isOpened() and rclpy.ok()):
        # フレームを読み込む
        ret, frame = cap.read()
        if ret == True:
            drone_obj.detectMarkers(frame)
            drone_obj.getDronePoint()
            drone_obj.getNextTarget(frame)
            drone_obj.getMovingDerection()
            frame = drone_obj.OverImage(frame)
            drone_obj.sendCommand()
            cv2.imshow('Webcam Live', frame)
            # writer.write(frame)
            rclpy.spin_once(drone_obj)
            # 'q'キーが押されたらループから抜ける
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
                break

    # キャプチャをリリースし、ウィンドウを閉じる
    # writer.release()
    cap.release()
    cv2.destroyAllWindows()
    # ros node破棄
    rclpy.shutdown()

if __name__ == '__main__':
    main()
