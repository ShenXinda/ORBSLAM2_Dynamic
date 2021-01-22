import sys
import os
import numpy as np
def safe_float(number):
    try:
        return float(number)
    except:
        return None

f = open("./evaluate/temp.txt")
line = f.readline()
datas=[]
while line:
    line=line.strip('\n')
    data = line.split(' ')
    datas.append(list(map(safe_float, data)))
    line = f.readline()

f.close()
datas = np.array(datas)
out = np.mean(datas,axis=0)
print('{:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.3f} {:.1f}'.format(out[1],out[2],out[3],out[4],out[5],out[6],out[7]/out[8],1000*out[9]))
