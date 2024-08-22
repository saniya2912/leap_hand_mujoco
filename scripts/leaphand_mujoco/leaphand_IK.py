import PyKDL as kdl
from urdf_parser_py.urdf import URDF

def find_joint_for_link(urdf_model, child_link_name):
    """ Find the joint in the URDF model that connects to the given child link. """
    for joint in urdf_model.joints:
        if joint.child == child_link_name:
            return joint
    return None

def add_joint_to_chain(chain, joint):
    if joint.type == 'revolute' or joint.type == 'continuous':
        kdl_joint = kdl.Joint(
            joint.name,
            kdl.Vector(joint.origin.xyz[0], joint.origin.xyz[1], joint.origin.xyz[2]),
            kdl.Vector(joint.axis[0], joint.axis[1], joint.axis[2]),
            kdl.Joint.RotAxis
        )
    elif joint.type == 'fixed':
        kdl_joint = kdl.Joint(joint.name, kdl.Joint.Fixed)
    else:
        print(f"Unsupported joint type: {joint.type}")
        return False

    kdl_segment = kdl.Segment(joint.child, kdl_joint, kdl.Frame())
    chain.addSegment(kdl_segment)
    return True

def create_kdl_chain_from_urdf(urdf_model, base_link, end_link):
    chain = kdl.Chain()
    current_link = end_link

    while current_link != base_link:
        joint = find_joint_for_link(urdf_model, current_link)
        if not joint:
            print(f"Joint for link {current_link} not found!")
            return None
        if not add_joint_to_chain(chain, joint):
            print(f"Failed to add joint {joint.name} to chain")
            return None
        current_link = joint.parent

    return chain

def perform_ik(chain, target_frame):
    num_joints = chain.getNrOfJoints()
    joint_positions = kdl.JntArray(num_joints)
    ik_solver = kdl.ChainIkSolverPos_LMA(chain)

    initial_positions = kdl.JntArray(num_joints)  # Start with zero positions
    result = ik_solver.CartToJnt(initial_positions, target_frame, joint_positions)

    if result >= 0:
        return [joint_positions[i] for i in range(num_joints)]
    else:
        raise RuntimeError("IK solver failed")

def main():
    urdf_path = '/home/sysidea/leap_hand_mujoco/model/leap hand/robot2.urdf'
    urdf_model = URDF.from_xml_file(urdf_path)

    # fingers = {
    #     'fingertip': kdl.Frame(kdl.Rotation(0.43244, 0.901663, 0,
    #                                         -0.901663, 0.43244, 0,
    #                                         0, 0, 1),
    #                         kdl.Vector(0.0047203, 0.00491587, 0)),
    #     'fingertip_2': kdl.Frame(kdl.Rotation(0.453596, 0.891207, 0,
    #                                         -0.891207, 0.453596, 0,
    #                                         0, 0, 1),
    #                             kdl.Vector(-0.00440944, 0.00226858, 0)),
    #     'fingertip_3': kdl.Frame(kdl.Rotation(0.696707, 0.717356, 0,
    #                                         -0.717356, 0.696707, 0,
    #                                         0, 0, 1),
    #                             kdl.Vector(0.010258, -0.00390178, 0)),
    #     'thumb_fingertip': kdl.Frame(kdl.Rotation(0.764842, 0.644218, 0,
    #                                             -0.644218, 0.764842, 0,
    #                                             0, 0, 1),
    #                                 kdl.Vector(-0.0154372, -0.00293029, 0))
    # }


    fingers = {
        'fingertip': kdl.Frame(kdl.Rotation.RPY(0, 0, 0),
                            kdl.Vector(0, 0, 0)),
        'fingertip_2': kdl.Frame(kdl.Rotation.RPY(0, 0, 0),
                                kdl.Vector(0, 0, 0)),
        'fingertip_3': kdl.Frame(kdl.Rotation.RPY(0, 0, 0),
                                kdl.Vector(0, 0, 0)),
        'thumb_fingertip': kdl.Frame(kdl.Rotation.RPY(0, 0, 0),
                                    kdl.Vector(0, 0, 0))
    }


    base_link = 'palm_lower'
    for finger, target_pose in fingers.items():
        chain = create_kdl_chain_from_urdf(urdf_model, base_link, finger)
        # if chain and chain.getNrOfSegments() > 0:
        #     #print(f"Chain for {finger} created with {chain.getNrOfSegments()} segments.")
        #     try:
        #         joint_positions = perform_ik(chain, target_pose)
        #         #print(f"Joint positions for {finger}: {joint_positions}")
        #     except RuntimeError as e:
        #         pass
        #         #print(f"IK failed for {finger}: {e}")
        # else:
        #     print(f"Chain for {finger} could not be created.")

    print(chain)

if __name__ == "__main__":
    main()



# SUCCESSFUL KDL CHAINING 

# import PyKDL as kdl
# from urdf_parser_py.urdf import URDF

# def find_joint_for_link(urdf_model, child_link_name):
#     """ Find the joint in the URDF model that connects to the given child link. """
#     for joint in urdf_model.joints:
#         if joint.child == child_link_name:
#             return joint
#     return None

# def add_joint_to_chain(chain, joint):
#     if joint.type == 'revolute' or joint.type == 'continuous':
#         kdl_joint = kdl.Joint(
#             joint.name,
#             kdl.Vector(joint.origin.xyz[0], joint.origin.xyz[1], joint.origin.xyz[2]),
#             kdl.Vector(joint.axis[0], joint.axis[1], joint.axis[2]),
#             kdl.Joint.RotAxis
#         )
#     elif joint.type == 'fixed':
#         kdl_joint = kdl.Joint(joint.name, kdl.Joint.Fixed)
#     else:
#         print(f"Unsupported joint type: {joint.type}")
#         return False

#     kdl_segment = kdl.Segment(joint.child, kdl_joint, kdl.Frame())
#     chain.addSegment(kdl_segment)
#     return True

# def create_kdl_chain_from_urdf(urdf_model, base_link, end_link):
#     chain = kdl.Chain()
#     current_link = end_link

#     while current_link != base_link:
#         joint = find_joint_for_link(urdf_model, current_link)
#         if not joint:
#             print(f"Joint for link {current_link} not found!")
#             return None
#         if not add_joint_to_chain(chain, joint):
#             print(f"Failed to add joint {joint.name} to chain")
#             return None
#         current_link = joint.parent

#     return chain

# def main():
#     urdf_path = '/home/barat/mujoco_leaphand/leap_right/robot.urdf'
#     urdf_model = URDF.from_xml_file(urdf_path)

#     fingers = [
#         {'base_link': 'palm_lower', 'end_link': 'fingertip'},
#         {'base_link': 'palm_lower', 'end_link': 'fingertip_2'},
#         {'base_link': 'palm_lower', 'end_link': 'fingertip_3'},
#         {'base_link': 'palm_lower', 'end_link': 'thumb_fingertip'},
#     ]

#     for finger in fingers:
#         chain = create_kdl_chain_from_urdf(urdf_model, finger['base_link'], finger['end_link'])
#         if chain:
#             print(f"Chain for {finger['end_link']} created with {chain.getNrOfSegments()} segments.")
#         else:
#             print(f"Chain for {finger['end_link']} could not be created.")

# if __name__ == "__main__":
#     main()
