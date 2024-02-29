import cv2
import mediapipe as mp
import numpy as np
import time
import serial

# Initialize pserial
esp32_com_port = 'COM12'  
baud_rate = 115200
ser = serial.Serial(esp32_com_port, baud_rate, timeout=1)

mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils
drawing_spec = mp_drawing.DrawingSpec(color=(255, 100, 50), thickness=1, circle_radius=1)
connection_drawing_spec = mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=1, circle_radius=1)  
cap = cv2.VideoCapture(0)

round_x = 0
serial_time = time.time()
while cap.isOpened():
    success, image = cap.read()
    start = time.time()

    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)  #flip BGRtoRGB
    image.flags.writeable = False
    results = face_mesh.process(image) #cam frame
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) #RGBtoBGR 

    img_h, img_w, img_c = image.shape
    face_3d = []
    face_2d = []

    if results.multi_face_landmarks:
        for face_landmarks in results.multi_face_landmarks:
            for idx, lm in enumerate(face_landmarks.landmark):
                if idx == 33 or idx == 263 or idx == 1 or idx == 61 or idx == 291 or idx == 199:
                    if idx == 1:
                        nose_2d = (lm.x * img_w, lm.y * img_h)
                        nose_3d = (lm.x * img_w, lm.y * img_h, lm.z * 3000)

                    x, y = int(lm.x * img_w), int(lm.y * img_h)
                    face_2d.append([x, y])
                    face_3d.append([x, y, lm.z])       #3d cordl
            

            face_2d = np.array(face_2d, dtype=np.float64) #2d array
            face_3d = np.array(face_3d, dtype=np.float64) #3d array


            focal_length = 1 * img_w

            cam_matrix = np.array([ [focal_length, 0, img_h / 2],
                                    [0, focal_length, img_w / 2],
                                    [0, 0, 1]])

            dist_matrix = np.zeros((4, 1), dtype=np.float64)

            success, rot_vec, trans_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, dist_matrix)
            rmat, jac = cv2.Rodrigues(rot_vec)
            angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(rmat) #for angles

            # Get the rotation degree
            x_angle = angles[0] * 360
            y_angle = angles[1] * 360
            z_angle = angles[2] * 360
          
            if y_angle < -10:
                text = "Looking Left"
            elif y_angle > 10:
                text = "Looking Right"
            elif x_angle < -10:
                text = "Looking Down"
            else:
                text = "Forward"
            
            round_y = round(y_angle, 2)
            round_x = round(x_angle, 2)
            mode = 1100

            if ((time.time() - serial_time) > 0.2):
                ser.write(f"{round_x},{round_y},{mode}\n".encode())
                serial_time = time.time()

            # Add the text on the image
            cv2.putText(image, text, (20, 450), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
            cv2.putText(image, "x: " + str(round_y), (500, 450), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
            #cv2.putText(image, "y: " + str(round_x), (500, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)

        mp_drawing.draw_landmarks(
                    image=image,
                    landmark_list=face_landmarks,
                    connections=mp_face_mesh.FACEMESH_CONTOURS,
                    landmark_drawing_spec=drawing_spec,
                    connection_drawing_spec=connection_drawing_spec)

    cv2.imshow('Head Pose Estimation', image)

    if cv2.waitKey(5) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
