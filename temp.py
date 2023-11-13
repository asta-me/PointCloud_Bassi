import numpy as np

a = np.array([4,2,5])

b = np.array([[5,6,7],[4,2,5],[5,5,5]])

print(b)
a == b[1,:]
#print(a  b)
print((a == b[0,:]).all())

def is_subarray(A, B):
    for subB in B:
        print(subB)
        if (A == subB).all():
            return True
    return False

print (is_subarray(a,b))

print(b[1,2,:])