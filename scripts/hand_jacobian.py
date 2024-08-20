import numpy as np
from scipy.linalg import block_diag

# contact frame wrt world
contact_orientation = [
    np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]]),
    np.array([[2, 3, 4], [5, 6, 7], [8, 9, 10]]),
    np.array([[3, 4, 5], [6, 7, 8], [9, 10, 11]]),
    np.array([[4, 5, 6], [7, 8, 9], [10, 11, 12]])
]
Rpki = [
    np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]]),
    np.array([[2, 3, 4], [5, 6, 7], [8, 9, 10]]),
    np.array([[3, 4, 5], [6, 7, 8], [9, 10, 11]]),
    np.array([[4, 5, 6], [7, 8, 9], [10, 11, 12]])
]

J_i = [
    np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]]),
    np.array([[2, 3, 4], [5, 6, 7], [8, 9, 10]]),
    np.array([[3, 4, 5], [6, 7, 8], [9, 10, 11]]),
    np.array([[4, 5, 6], [7, 8, 9], [10, 11, 12]])
]

Jh_blocks=[]
n=4

for i in range(n):
    Jh_i=np.matmul(np.matmul(contact_orientation[i].T,Rpki[i]),J_i[i])
    Jh_blocks.append(Jh_i)

Jh = block_diag(*Jh_blocks)
print(Jh)
