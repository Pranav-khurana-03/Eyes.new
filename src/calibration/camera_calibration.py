import cv2
import glob
import numpy as np

pathL = "./calibration_images/left_images"
pathR = "./calibration_images/right_images"


criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)


def main():

    img_ptsL = []
    img_ptsR = []
    obj_pts = []

    images = glob.glob(pathL + "/*.jpg")

    for i in range(0,11):
        imgL = cv2.imread(pathL+"calb_img_%d.png"%i)
        imgR = cv2.imread(pathR+"calb_img_%d.png"%i)
        imgL_gray = cv2.imread(pathL+"calb_img_%d.png"%i,0)
        imgR_gray = cv2.imread(pathR+"calb_img_%d.png"%i,0)

        outputL = imgL.copy()
        outputR = imgR.copy()

        retR, cornersR =  cv2.findChessboardCorners(outputR,(9,6),None)
        retL, cornersL = cv2.findChessboardCorners(outputL,(9,6),None)

        if retR and retL:
            obj_pts.append(objp)
            cv2.cornerSubPix(imgR_gray,cornersR,(11,11),(-1,-1),criteria)
            cv2.cornerSubPix(imgL_gray,cornersL,(11,11),(-1,-1),criteria)
            cv2.drawChessboardCorners(outputR,(9,6),cornersR,retR)
            cv2.drawChessboardCorners(outputL,(9,6),cornersL,retL)
            cv2.imshow('cornersR',outputR)
            cv2.imshow('cornersL',outputL)
            cv2.waitKey(0)

            img_ptsL.append(cornersL)
            img_ptsR.append(cornersR)
            

    retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(obj_pts,img_ptsL,imgL_gray.shape[::-1],None,None)
    hL,wL= imgL_gray.shape[:2]
    new_mtxL, roiL= cv2.getOptimalNewCameraMatrix(mtxL,distL,(wL,hL),1,(wL,hL))

    retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(obj_pts,img_ptsR,imgR_gray.shape[::-1],None,None)
    hR,wR= imgR_gray.shape[:2]
    new_mtxR, roiR= cv2.getOptimalNewCameraMatrix(mtxR,distR,(wR,hR),1,(wR,hR))


    flags = 0
    flags |= cv2.CALIB_FIX_INTRINSIC

    criteria_stereo= (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


    retS, new_mtxL, distL, new_mtxR, distR, Rot, Trns, Emat, Fmat = cv2.stereoCalibrate(obj_pts, img_ptsL, img_ptsR, new_mtxL, distL, new_mtxR, distR, imgL_gray.shape[::-1], criteria_stereo, flags)


    rectify_scale= 1
    rect_l, rect_r, proj_mat_l, proj_mat_r, Q, roiL, roiR= cv2.stereoRectify(new_mtxL, distL, new_mtxR, distR,imgL_gray.shape[::-1], Rot, Trns, rectify_scale,(0,0))

    Left_Stereo_Map = cv2.initUndistortRectifyMap(new_mtxL, distL, rect_l, proj_mat_l, imgL_gray.shape[::-1], cv2.CV_16SC2)
    Right_Stereo_Map = cv2.initUndistortRectifyMap(new_mtxR, distR, rect_r, proj_mat_r, imgR_gray.shape[::-1], cv2.CV_16SC2)

    cv_file = cv2.FileStorage("params.xml", cv2.FILE_STORAGE_WRITE)
    cv_file.write("Left_Stereo_Map_x",Left_Stereo_Map[0])
    cv_file.write("Left_Stereo_Map_y",Left_Stereo_Map[1])
    cv_file.write("Right_Stereo_Map_x",Right_Stereo_Map[0])
    cv_file.write("Right_Stereo_Map_y",Right_Stereo_Map[1])
    cv_file.release()



if __name__ == "__main__":
    main()