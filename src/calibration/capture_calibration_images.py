import glob
import os
import cv2
import numpy as np

output_dir = "calibration_images"

left_dir = "left_images"
right_dir = "right images"

def main():

    left_image = cv2.VideoCapture(0)
    right_image = cv2.VideoCapture(1)

    os.mkdir(output_dir)
    os.mkdir(output_dir + "/" + left_dir)
    os.mkdir(output_dir + "/" + right_dir)

    i=0 

    while True:
        retL, frameL = left_image.read()
        relR, frameR = right_image.read()

        frameL = format(frameL)
        frameR = format(frameR)
        
        img = cv2.vconcat([frameL, frameR]) 
        cv2.imshow("output", img)


        if cv2.waitKey(1) & 0xFF == ord('s'):
            cv2.imwrite(output_dir + "/" + left_dir + "/calb_img_"+str(i)+".jpg", frameL)
            cv2.imwrite(output_dir + "/" + right_dir + "/calb_img_"+str(i)+".jpg", frameR)
            i+=1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


def format(frame):
    frame = ((frame / np.max(frame)) * 255).astype('uint8')
    row, col, _ = frame.shape
    _max = max(col, row)
    result = np.zeros((_max, _max, 3), np.uint8)
    result[0:row, 0:col] = frame
    result.shape
    return result


if __name__ == "__main__":
    main()