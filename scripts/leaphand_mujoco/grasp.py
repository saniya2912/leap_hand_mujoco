# import numpy as np

# n=4

# #Body wrt world
# r_theta_list = [
#     np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]]),
#     np.array([[2, 3, 4], [5, 6, 7], [8, 9, 10]]),
#     np.array([[3, 4, 5], [6, 7, 8], [9, 10, 11]]),
#     np.array([[4, 5, 6], [7, 8, 9], [10, 11, 12]])
# ]

# # contact frame wrt world
# contact_orientation_list = [
#     np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]]),
#     np.array([[2, 3, 4], [5, 6, 7], [8, 9, 10]]),
#     np.array([[3, 4, 5], [6, 7, 8], [9, 10, 11]]),
#     np.array([[4, 5, 6], [7, 8, 9], [10, 11, 12]])
# ]

# b_i=[[0],[0],[0]]

# def G_i(contact_orientation, r_theta):
#     matrix1 = contact_orientation
#     r_theta_b = np.dot(r_theta, b_i)  # Matrix multiplication
    
#     matrix2 = np.array([np.cross(r_theta_b.flatten(), contact_orientation[:, 0].flatten()),
#                         np.cross(r_theta_b.flatten(), contact_orientation[:, 1].flatten()),
#                         np.cross(r_theta_b.flatten(), contact_orientation[:, 2].flatten())])
    
#     return np.vstack([matrix1, matrix2])

# # Initialize an empty list to hold all G_i matrices
# G_matrices = []

# for i in range(n):
#     G_i_matrix = G_i(contact_orientation_list[i], r_theta_list[i])
#     G_matrices.append(G_i_matrix)

# # Concatenate all G_i matrices horizontally to form G
# G = np.hstack(G_matrices)

# print("G matrix:")
# print(G)

import numpy as np


class GraspClass:
    def __init__(self):
        self.G_matrices=[]
        
    def G_i(self,contact_orientation, r_theta,b):
        matrix1 = contact_orientation
        r_theta_b = np.dot(r_theta,b)  # Matrix multiplication
        
        matrix2 = np.array([np.cross(r_theta_b.flatten(), contact_orientation[:, 0].flatten()),
                            np.cross(r_theta_b.flatten(), contact_orientation[:, 1].flatten()),
                            np.cross(r_theta_b.flatten(), contact_orientation[:, 2].flatten())])
        
        return np.vstack([matrix1, matrix2])


    def G(self,n,contact_orientations, r_theta,bs):
        for i in range(n):
            G_i_matrix = self.G_i(contact_orientations[i],r_theta,bs[i])
            self.G_matrices.append(G_i_matrix)

        # Concatenate all G_i matrices horizontally to form G
        G = np.hstack(self.G_matrices)
        return G



