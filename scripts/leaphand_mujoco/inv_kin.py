import numpy as np
import mujoco
import mujoco.viewer as viewer
import mediapy as media



class GradientDescentIK:
    def __init__(self,xml_path):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.jacp = np.zeros((3, self.model.nv))  # translation jacobian
        self.jacr = np.zeros((3, self.model.nv)) 
        self.step_size = 0.5
        self.tol = 0.01
        self.alpha = 0.5
        self.init_q = [0.0, 0.0, 0.0, 0.0]  
    
    def check_joint_limits(self, q):
        """Check if the joints are under or above their limits."""
        for i in range(len(q)):
            q[i] = max(self.model.jnt_range[i][0], min(q[i], self.model.jnt_range[i][1]))


    def calculate(self, goal_pos, goal_rot, site_name):
        self.data.qpos = self.init_q
        mujoco.mj_forward(self.model, self.data)

        site_id= self.model.site(site_name).id
        
        # Current pose and orientation
        current_pos = self.data.site(site_id).xpos
        current_rot = self.data.site(site_id).xmat.reshape(3, 3)

        # Position and orientation error
        pos_error = np.subtract(goal_pos, current_pos)
        rot_error = 0.5 * (np.cross(current_rot[:, 0], goal_rot[:, 0]) +
                           np.cross(current_rot[:, 1], goal_rot[:, 1]) +
                           np.cross(current_rot[:, 2], goal_rot[:, 2]))

        # Combine position and orientation errors
        error = np.concatenate([pos_error, rot_error])

        max_iterations = 100000
        iteration = 0

        while np.linalg.norm(error) >= self.tol and iteration < max_iterations:
            # Calculate Jacobian for position and orientation
            mujoco.mj_jacSite(self.model, self.data, self.jacp, self.jacr, site_id)
            full_jacobian = np.vstack((self.jacp, self.jacr))
            
            # Calculate gradient
            grad = self.alpha * full_jacobian.T @ error
            
            # Compute next step
            self.data.qpos += self.step_size * grad
            
            # Check joint limits
            self.check_joint_limits(self.data.qpos)
            
            # Compute forward kinematics
            mujoco.mj_forward(self.model, self.data)
            
            # Update position and orientation error
            current_pos = self.data.site(site_id).xpos
            current_rot = self.data.site(site_id).xmat.reshape(3, 3)
            pos_error = np.subtract(goal_pos, current_pos)
            rot_error = 0.5 * (np.cross(current_rot[:, 0], goal_rot[:, 0]) +
                               np.cross(current_rot[:, 1], goal_rot[:, 1]) +
                               np.cross(current_rot[:, 2], goal_rot[:, 2]))
            error = np.concatenate([pos_error, rot_error])

            iteration += 1

        if iteration >= max_iterations:
            print("Warning: Maximum iterations reached. The solution may not have converged.")
        
        result = self.data.qpos.copy()
        return result
    
class OnlyPosIK:
    def __init__(self,xml_path):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.jacp = np.zeros((3, self.model.nv))  # translation jacobian
        self.jacr = np.zeros((3, self.model.nv)) 
        self.step_size = 0.5
        self.tol = 0.01
        self.alpha = 0.5
        self.init_q = [0.0, 0.0, 0.0, 0.0]  
    
    def check_joint_limits(self, q):
        """Check if the joints are under or above their limits."""
        for i in range(len(q)):
            q[i] = max(self.model.jnt_range[i][0], min(q[i], self.model.jnt_range[i][1]))


    def calculate(self, goal_pos,site_name):
        self.data.qpos = self.init_q
        mujoco.mj_forward(self.model, self.data)

        site_id= self.model.site(site_name).id
        
        # Current pose and orientation
        current_pos = self.data.site(site_id).xpos
        # Position and orientation error
        pos_error = np.subtract(goal_pos, current_pos)

        # Combine position and orientation errors
        error = pos_error

        max_iterations = 100000
        iteration = 0

        while np.linalg.norm(error) >= self.tol and iteration < max_iterations:
            # Calculate Jacobian for position and orientation
            mujoco.mj_jacSite(self.model, self.data, self.jacp, self.jacr, site_id)
            # full_jacobian = np.vstack((self.jacp, self.jacr))
            
            # Calculate gradient
            grad = self.alpha * self.jacp.T @ error
            
            # Compute next step
            self.data.qpos += self.step_size * grad
            
            # Check joint limits
            self.check_joint_limits(self.data.qpos)
            
            # Compute forward kinematics
            mujoco.mj_forward(self.model, self.data)
            
            # Update position and orientation error
            current_pos = self.data.site(site_id).xpos
            # current_rot = self.data.site(site_id).xmat.reshape(3, 3)
            pos_error = np.subtract(goal_pos, current_pos)
            # rot_error = 0.5 * (np.cross(current_rot[:, 0], goal_rot[:, 0]) +
            #                    np.cross(current_rot[:, 1], goal_rot[:, 1]) +
            #                    np.cross(current_rot[:, 2], goal_rot[:, 2]))
            # error = np.concatenate([pos_error, rot_error])

            iteration += 1

        if iteration >= max_iterations:
            print("Warning: Maximum iterations reached. The solution may not have converged.")
        
        result = self.data.qpos.copy()
        return result

   

# # Init variables
#IK_index=GradientDescentIK('/home/sysidea/leap_hand_mujoco/model/leap hand/index_finger.xml')
# IK_thumb=GradientDescentIK('/home/sysidea/leap_hand_mujoco/model/leap hand/thumb.xml')
# body_id_index = IK_index.model.body('fingertip').id
# body_id_thumb = IK_index.model.body('thumb_fingertip_4').id

# goal_pos_index = [0.0565407, 0.0226578, -0.07722874]  # desired position of the end effector
# goal_rot_index=np.array([ [1.00000000e+00, -3.92523115e-16,  0.00000000e+00],  [0.00000000e+00,
#   0.00000000e+00, -1.00000000e+00],  [3.92523115e-16,  1.00000000e+00,
#   0.00000000e+00]])
# init_q = [0.0, 0.0, 0.0, 0.0]  # initial joint configuration



# camera = mujoco.MjvCamera()
# mujoco.mjv_defaultFreeCamera(IK_index.model, camera)

# # Set camera position
# camera.lookat[:] = np.array([0.0, 0.0, 0.0])  # Point in space the camera is looking at
# camera.azimuth = 135                   # Horizontal angle in degrees
# camera.elevation = -10                      # Vertical angle in degrees
# camera.distance = 1                         # Distance from the lookat point

# with mujoco.Renderer(IK_index.model) as renderer:
#     # Reset qpos to the initial keyframe
#     mujoco.mj_resetDataKeyframe(IK_index.model, IK_index.data, 1)

#     # Calculate the desired joint angles using the IK solver
# result= IK_index.calculate(goal_pos_index, goal_rot_index,'fingertip')
# print(result)
#     # # Get the result after IK calculation
#     # result = IK.data.qpos.copy()

#     # Visualize the result after Gradient Descent IK
#     IK_index.data.qpos = result  # set the configuration to the IK result
#     mujoco.mj_forward(IK_index.model, IK_index.data)
#     result_point = IK_index.data.body(body_id_index).xpos.copy()  # store the resultant position
#     renderer.update_scene(IK_index.data, camera)
#     result_plot = renderer.render()

#     # Print positions
#     print("Testing point =>", goal_pos)
#     print("Gradient Descent result =>", result_point, "\n")

#     # Display images
#     images = {
#         'Gradient Descent result': result_plot,
#     }

#     media.show_images(images)