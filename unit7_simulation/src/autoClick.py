import pyautogui
import time
import signal
import sys

def find_and_click(image1_path, image2_path, confidence=0.8,current_point=''):
    global old_point
    try:
        # ค้นหารูปภาพบนหน้าจอด้วย confidence ที่กำหนด
        image1_location = pyautogui.locateCenterOnScreen(image1_path, confidence=confidence)
        image2_location = pyautogui.locateCenterOnScreen(image2_path, confidence=0.9)
        
        # ถ้าพบรูปที่ 1 และรูปที่ 2 บนหน้าจอ
        if image1_location and image2_location:
        # if image2_location:
            # เลื่อนเมาส์ไปที่ตำแหน่งที่ต้องการคลิก
            pyautogui.moveTo(image1_location, duration=1)
            # คลิกเม้าซ้ายสองครั้ง
            pyautogui.click(clicks=1)
            # เลื่อนเมาส์ไปตำแหน่ง 100, 100
            pyautogui.moveTo(10, 10)
            print("go to ",image1_path)
            time.sleep(1)
            old_point= current_point
        image1_location = False
        image2_location = False
    except pyautogui.ImageNotFoundException:
        # print(f"Could not locate the image: {image1_path} or {image2_path}")
        pass

def signal_handler(sig, frame):
    print('Program terminated by user')
    sys.exit(0)

if __name__ == "__main__":
    # ตัวอย่างการเรียกใช้งานฟังก์ชัน
    imageHOME_path = '/home/ros3/sim_ws/src/agv_pkg/src/autoClickImage/HOME.png'
    imageA_path = '/home/ros3/sim_ws/src/agv_pkg/src/autoClickImage/A.png'
    imageB_path = '/home/ros3/sim_ws/src/agv_pkg/src/autoClickImage/B.png'
    imageC_path = '/home/ros3/sim_ws/src/agv_pkg/src/autoClickImage/C.png'
    imageD_path = '/home/ros3/sim_ws/src/agv_pkg/src/autoClickImage/D.png'
    imageE_path = '/home/ros3/sim_ws/src/agv_pkg/src/autoClickImage/E.png'
    imageWaiting_cmd_path = '/home/ros3/sim_ws/src/agv_pkg/src/autoClickImage/waiting_cmd.png'
    image_suc_Home_path = '/home/ros3/sim_ws/src/agv_pkg/src/autoClickImage/suc_HOME.png'
    image_suc_A_path = '/home/ros3/sim_ws/src/agv_pkg/src/autoClickImage/suc_A.png'
    image_suc_B_path = '/home/ros3/sim_ws/src/agv_pkg/src/autoClickImage/suc_B.png'
    image_suc_C_path = '/home/ros3/sim_ws/src/agv_pkg/src/autoClickImage/suc_C.png'
    image_suc_D_path = '/home/ros3/sim_ws/src/agv_pkg/src/autoClickImage/suc_D.png'
    image_suc_E_path = '/home/ros3/sim_ws/src/agv_pkg/src/autoClickImage/suc_E.png'

    # จัดการสัญญาณ SIGINT (Ctrl+C)
    signal.signal(signal.SIGINT, signal_handler)
    global old_point
    old_point = input("Enter current_point (H, A, B, C, D, E): ").upper()
    while True:
        try:
            # print(old_point)
            if old_point == "H":
                find_and_click(imageA_path, imageWaiting_cmd_path, confidence=0.8,current_point="A")
                time.sleep(1) 
                find_and_click(imageA_path, image_suc_Home_path, confidence=0.8,current_point="A")
                time.sleep(1)
            elif old_point == "A":
                find_and_click(imageB_path, image_suc_A_path, confidence=0.8,current_point="B")
                time.sleep(1) 
 
            elif old_point == "B":
                find_and_click(imageC_path, image_suc_B_path, confidence=0.8,current_point="C")
                time.sleep(1) 

            elif old_point == "C":
                find_and_click(imageD_path, image_suc_C_path, confidence=0.8,current_point="D")
                time.sleep(1)

            elif old_point == "D":
                find_and_click(imageE_path, image_suc_D_path, confidence=0.8,current_point="E")
                time.sleep(1)

            elif old_point == "E":
                find_and_click(imageHOME_path, image_suc_E_path, confidence=0.8,current_point="H")
                time.sleep(1)  # รอ 1 วินาทีก่อนทำงานซ้ำ

        except Exception as e:
            print(f"An error occurred: {e}")
