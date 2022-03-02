
# You can dump an image as a numpy array and manipulate the pixel values that way.

from PIL import Image
import numpy as np
im=Image.open("captured.png")
pixels=np.asarray(im.getdata())
print(pixels)

st = str(pixels)
f = open("/home/kartikey/Kartikey_swarm/draw"+".txt",'w+')
f.write(st)
