import numpy as np
import cv2
import os

os.chdir('./Captures')
k = input('Enter the name of file to you want to undistort: ')
nameOnly = k.rsplit('.')[0]
img = cv2.imread(k)
    

# Define Camera Matrix
mtx = np.loadtxt('cameraMatrix.txt', delimiter=',')


# Define distortion coefficients
dist = np.loadtxt('cameraDistortion.txt', delimiter=',')

# make a new directory to save undistorted images to if it does not exist already
try:
    os.mkdir('../Undistorted_Images')
except FileExistsError:
    print('Undistorted_Images directory already exists')
else:
    print('Made new directory: Undistorted_Images')

# Undistort
dst = cv2.undistort(img, mtx, dist, None, mtx)
cv2.imwrite('../Undistorted_Images/'+ nameOnly + '_Undistorted.png', dst)
print(f'Image {k} successfully undistorted')