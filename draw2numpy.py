#!/usr/bin/env python3

import sys
import numpy as np
from PIL import Image, ImageFilter,ImageOps

# Check an image file was supplied
if len(sys.argv) != 2:
    print("Usage: curve2array IMAGE", file=sys.stderr)
    exit(1)

# Assign filename and open image as greyscale
filename = sys.argv[1]
im = Image.open(filename).convert('L')
# inverts the color so the background is black and the drawing is white
im = ImageOps.invert(im)
# thresh = 200
# fn = lambda x : 255 if x > thresh else 0
# im = im.convert('L').point(fn, mode='1')
# r.save('foo.png')
# OPTIONAL = Blur the image a little to allow discontinuities in hand-sketched curves
# im = im.filter(ImageFilter.GaussianBlur(1))
# converts to pure black and white
# im = im.convert('1')

# Convert image to Numpy array and get row number of brightest
# pixel in each column
na    = np.asarray(im)
print("the img size is   "+str(np.size(na)))
yvals = np.argmax(im, axis=0)
yvals = na.shape[0] - yvals     # Make origin at bottom-left
# instead of top-left

# Write result as ".npy" file
np.save('result.npy', yvals)

print(yvals)
print("the size is ",np.size(yvals))
print(na[:][200])
print(np.size(na[15]))
im.show()
