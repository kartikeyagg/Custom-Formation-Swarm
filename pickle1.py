import pickle
import numpy as np
import matplotlib.pyplot as plt
# mylist = [2,3,1,6,7]
# with open('/home/kartikey/Kartikey_swarm/parrot.pkl', 'wb') as f:
#     pickle.dump(mylist, f)
#
# f.close()
with open('/home/kartikey/Kartikey_swarm/cart_coord_cursor.pkl', 'rb') as f:
    mynewlist = pickle.load(f)
print(mynewlist)

mynewlist = np.asarray(mynewlist)
plt.scatter(mynewlist[:,0],mynewlist[:,1])
plt.gca().set_aspect('equal',adjustable='box')

plt.show()

# list = [1,2,4]
# print(list[0:2])
