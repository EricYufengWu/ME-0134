# -*- coding: utf-8 -*-
"""Slim Letters to Cartesian.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/10bo1d23dcKYfypzJ4XQLGq3AII3bbzX5
"""

import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
import numpy as np
from PIL import Image
from PIL import ImageColor
# from resizeimage import resizeimage
XSize = 10
YSize = 10

image = Image.open('LetterG1.png')
new_image = image.resize((XSize, YSize))
new_image.save('image_400.png')

GImage = Image.open("image_400.png")
GImage_LA = GImage.convert("LA")

GX = []
GY = []
threshold = 100
for x in range(0, 9):
  for y in range(0, 9):
    G_pixel_value = GImage_LA.getpixel((x,y))
    if G_pixel_value[1] > threshold:
        GX.append(x)
        GY.append(-y)

plt.plot(GX, GY, 'o', color='black');



Fimage = Image.open('LetterF.png')
new_image = Fimage.resize((XSize, YSize))
new_image.save('image_300.png')

FImage = Image.open("image_300.png")
FImage_LA = FImage.convert("LA")

FX = []
FY = []
threshold = 50
for x in range(0, 9):
  for y in range(0, 9):
    F_pixel_value = FImage_LA.getpixel((x,y))
    if F_pixel_value[1] > threshold:
        FX.append(x)
        FY.append(-y)

plt.plot(FX, FY, 'o', color='black');



Eimage = Image.open('LetterE.png')
new_image = Eimage.resize((XSize, YSize))
new_image.save('image_200.png')

EImage = Image.open("image_200.png")
EImage_LA = EImage.convert("LA")

EX = []
EY = []
threshold = 100
for x in range(0, 9):
  for y in range(0, 9):
    E_pixel_value = EImage_LA.getpixel((x,y))
    if E_pixel_value[1] > threshold:
        EX.append(x)
        EY.append(-y)

plt.plot(EX, EY, 'o', color='black');



Wimage = Image.open('LetterW.png')
new_image = Wimage.resize((XSize, YSize))
new_image.save('image_100.png')

WImage = Image.open("image_100.png")
WImage_LA = WImage.convert("LA")

WX = []
WY = []
threshold = 50
for x in range(0, 9):
  for y in range(0, 9):
    W_pixel_value = WImage_LA.getpixel((x,y))
    if W_pixel_value[1] > threshold:
        WX.append(x)
        WY.append(-y)

plt.plot(WX, WY, 'o', color='black');

IdealWidth = 4
IdealLength = 4
XMult = IdealWidth/2/XSize
YMult = IdealLength/2/YSize
GX1 = [XMult*i for i in GX]
GY1 = [YMult*i for i in GY]
GX2 = np.array(GX1)
GX3 = GX2 - IdealWidth/2
GY2 = np.array(GY1)
GY3 = GY2 + IdealLength/2

FX1 = [XMult*i for i in FX]
FY1 = [YMult*i for i in FY]
FX2 = np.array(FX1)
FX3 = FX2
FY2 = np.array(FY1)
FY3 = FY2 + IdealLength/2

EX1 = [XMult*i for i in EX]
EY1 = [YMult*i for i in EY]
EX2 = np.array(EX1)
EX3 = EX2 - IdealWidth/2
EY2 = np.array(EY1)
EY3 = EY2

WX1 = [XMult*i for i in WX]
WY1 = [YMult*i for i in WY]
WX2 = np.array(WX1)
WX3 = WX2
WY2 = np.array(WY1)
WY3 = WY2

TotalX = []
TotalX.extend(GX3)
TotalX.extend(FX3)
TotalX.extend(EX3)
TotalX.extend(WX3)

TotalY = []
TotalY.extend(GY3)
TotalY.extend(FY3)
TotalY.extend(EY3)
TotalY.extend(WY3)

print(TotalX)

plt.plot(TotalX, TotalY, 'o', color='black');

