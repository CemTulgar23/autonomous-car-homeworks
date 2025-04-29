import cv2
import numpy as np


def binary_array(array, thresh, value=0):

  if value == 0:
    binary = np.ones_like(array)   
  else:
    binary = np.zeros_like(array)  

  binary[(array >= thresh[0]) & (array <= thresh[1])] = value
 
  return binary
 

def blur_gaussian(image, ksize=3):

  return cv2.GaussianBlur(image, (ksize, ksize), 0)
         

def mag_thresh(image, sobel_kernel=3, thresh=(0, 255)):

  sobelx = np.absolute(sobel(image, orient='x', sobel_kernel=sobel_kernel))
  sobely = np.absolute(sobel(image, orient='y', sobel_kernel=sobel_kernel))
  mag = np.sqrt(sobelx ** 2 + sobely ** 2)
  
  return binary_array(mag, thresh)
 

def sobel(image, orient='x', sobel_kernel=3):
  
  if orient == 'x':
    sobel = cv2.Sobel(image, cv2.CV_64F, 1, 0, sobel_kernel)
  if orient == 'y':
    sobel = cv2.Sobel(image, cv2.CV_64F, 0, 1, sobel_kernel)
 
  return sobel
 

def threshold(image, thresh=(128,255), thresh_type=cv2.THRESH_BINARY):

  return cv2.threshold(image, thresh[0], thresh[1], thresh_type)