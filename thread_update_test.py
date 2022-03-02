



import threading
import time

def update_i(A,i):

    A[i]+=i;


A = [0,0,0,0,0,0]
thread_list = []

for i in range (len(A)):
    thread_list.append(threading.Thread(target=update_i,args=(A,i)))
for i in (thread_list):
    i.start()

time.sleep(5)
for i in thread_list:
    i.join()
print(A)
