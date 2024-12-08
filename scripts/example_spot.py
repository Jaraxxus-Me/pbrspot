# This example is intended to show off functionality and provide examples
# I suppose its somewhat in the style of a Jupyter notebook... 
# but without that nice incremental sections! So it'll just run everything!  

import os
import numpy as np
import pbrspot

if __name__ == '__main__':
    # Launch pybullet
    pbrspot.utils.connect(use_gui=True)
    pbrspot.utils.disable_real_time()
    # update these information from ROS2 service localizer
    body_frame_x = 2.122
    body_frame_y = 1.03
    body_frame_z = 0.84
    body_rot = [0.0, 0.0, 1.0, 0.0] # qx qy qz qw
    pbrspot.utils.set_camera(180, -10, 2.5, [body_frame_x, body_frame_y, body_frame_z-0.74])
    # Create a rng with a seed so that executions stay
    # consistent between runs.
    rng = np.random.default_rng(0)

    # Create robot object 
    robot = pbrspot.spot.Spot()

    # Optionally, you could create just an arm, with no base. 
    #robot = pb_robot.spot.SpotArm()
    #robot.set_point([0, 0, 0])

    # Add floor object 
    curr_path = os.getcwd()
    models_path = os.path.join(curr_path, 'models')
    floor_file = os.path.join(models_path, 'short_floor.urdf')
    floor = pbrspot.body.createBody(floor_file)
    floor.set_point([0, 0, body_frame_z-0.84])
    # match the ROS2 service localizer
    robot.set_body_point([body_frame_x, body_frame_y, body_frame_z])
    robot.set_quat(body_rot)

    # Visualize body pose
    body_pose = robot.get_body_transform()
    print("Visualizing body pose! {}".format(pbrspot.geometry.pose_from_tform(body_pose)))
    pbrspot.viz.draw_pose(pbrspot.geometry.pose_from_tform(body_pose), length=0.4, width=5)
    # pbrspot.utils.wait_for_user()

    ####################
    # FK and IK Examples
    ####################

    # Get the configuration of the arm
    q = robot.arm.GetJointValues()
    print(f"Current robot arm joint values: {q}")

    # Compute the forward kinematics
    pose = robot.arm.ComputeFK(q)
    pose_vec = pbrspot.geometry.pose_from_tform(pose)
    # Check they are aligned with the ROS2 service localizer
    print(f"Current robot hand pose: {pose_vec}")
    pbrspot.viz.draw_pose(pbrspot.geometry.pose_from_tform(pose), length=0.4, width=5)

    # Note that you could also get the end effector (EE) pose directly 
    # (here pose == other_pose)
    other_pose = robot.arm.GetEETransform()

    # Slighly modify to generate a new pose
    pose[0, 3] -= 0.5
    print(f"Slightly-modified pose to move to: {pose}")

    # Visualize this new pose
    print("Visualizing modified pose!")
    pbrspot.viz.draw_pose(pbrspot.geometry.pose_from_tform(pose), length=0.5, width=10)
    pbrspot.utils.wait_for_user()

    # Compute IK for this new pose
    newq = robot.arm.ComputeIK(pose, rng)
    # Not that ComputeIK() returns None if the IK solver couldn't find a solution. 
    # So we'd always want to check if the solution was found before setting the configuration
    print("Moving to modified pose!")
    if newq is not None:
        robot.arm.SetJointValues(newq)
    # # Note that you can also ask for the IK solution that is closest to some seed configuration.
    # # Importantly, note that this can fail and still return None.
    # newq_seed = robot.arm.ComputeIK(pose, rng, seed_q=q)
    # robot.arm.SetJointValues(newq_seed)
    pbrspot.utils.wait_for_user()

    # By default the IK solver is using the current base pose. 
    # But we can compute the IK for a specific base pose. 
    # Here lets create a base pose that moved slightly over
    movebase_pose = robot.get_transform() # note that we will use world link now
    movebase_pose[1, 3] += 0.05
    # Then we can solve IK for this base pose
    movebase_q = robot.arm.ComputeIK(pose, rng, robot_worldF=movebase_pose)
    assert movebase_q is not None
    # Note that actual locomotion is not implemented: we simply teleport the base.
    robot.set_transform(movebase_pose)
    print("Teleporting base!")
    pbrspot.utils.wait_for_user()
    print("Moving hand to corresponding correct pose after base teleport!")
    robot.arm.SetJointValues(movebase_q)
    pbrspot.utils.wait_for_user()

    # We can also control the hand, which is just a revolute joint. 
    # We can open the hand, which sets it to one joint limit
    print("Opening gripper!")
    robot.arm.hand.Open()
    pbrspot.utils.wait_for_user()
    
    # We can close it, which sets it to the other joint limit
    print("Closing gripper!")
    robot.arm.hand.Close()
    pbrspot.utils.wait_for_user()
    # Or we can specify the value of the joint
    print("Specifying gripper joint value!")
    robot.arm.hand.MoveTo(-0.25)
    pbrspot.utils.wait_for_user()

    # Teleport robot back to origin
    robot.set_point([0, 0, 0])


    #############################
    # Collision Checking Examples
    #############################

    # Lets modify the pose such that its in the floor, bwahaha!
    # q = robot.arm.GetJointValues()
    # print(f"Current robot arm joint values: {q}")

    # # Compute the forward kinematics
    # pose = robot.arm.ComputeFK(q)
    # pose[2, 3] -= 0.8
    # # pose[0, 3] += 0.3
    # ninety_degree_pitch = np.array([[0.0000000,  0.0000000,  1.0000000, 0.0],
    #                         [0.0000000,  1.0000000,  0.0000000, 0.0],
    #                         [-1.0000000,  0.0000000,  0.0000000, 0.0],
    #                         [0.0, 0.0, 0.0, 1.0]])
    # pose = pose @ ninety_degree_pitch
    # # Visualize this new pose.
    # print("Visualizing new pose that's inside floor")
    # pbrspot.viz.draw_pose(pbrspot.geometry.pose_from_tform(pose), length=0.5, width=10)
    # pbrspot.utils.wait_for_user()

    # # Again, compute IK. Note that IK does not collision check
    # floorq = robot.arm.ComputeIK(pose, rng)
    # # By default, the collision checker checks against all objects in the environment. 
    # # We can also collision checks against a particular set of obstacles. In this example
    # # I can do something silly and collision check against a empty set of obstacles. 
    # # Hence withFloor is in collision but withoutFloor isn't.
    # assert floorq is not None
    # withFloor = robot.arm.IsCollisionFree(floorq)
    # withoutFloor = robot.arm.IsCollisionFree(floorq, obstacles=[])
    # print(f"Check against floor during move? {withFloor}. Check without floor during move? {withoutFloor}")
    # pbrspot.utils.wait_for_user()

    ##########################
    # Motion Planning Examples 
    ##########################

    # Get a random configuration. By default, the configuration is collision
    # checked against all objects in the environment (similar to IsCollisionFree(), 
    # you can optionally pass in the set of obstacles to collision check against). 
    qrand = robot.arm.randomConfiguration(rng)

    # Plan a collision-free joint space path from q to qrand
    # Note that this is my python, non-optimized implementation, so its not the 
    # fastest planners. Better engineers write fast planners, so blame 
    # the speed on me, not sample-based planning in general :) 
    print("Visualizing Bi-RRT planning to random configuration.")
    path = robot.arm.birrt.PlanToConfiguration(robot.arm, q, qrand)
    # The path is a list of joint configurations (or None, if no path is found)
    # ExecutePositionPath just visualizes the path by teleporting between the 
    # positions - its a very dumb function. 
    # I do have most of the code to run path execution with pybullet physics, 
    # I'd just have to hook it up better
    pbrspot.utils.wait_for_user()
    print("Path found! Teleporting arm to configuration!")
    assert path is not None
    robot.arm.ExecutePositionPath(path)
    
    # Close out Pybullet
    pbrspot.utils.disconnect()
