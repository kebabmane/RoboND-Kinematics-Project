## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./assets/DH_Params.png
[image2]: ./assets/KUKA_Arm.png
[image3]: ./assets/Joint_locations.png
[image4]: ./assets/Inverse-kinematics.png


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! - and I edited it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

I derived the DH Params from the below axis assignments (sorry for the hand writing!)

![Kuka ARM][image2]

Then I extracted the values as displayed below from the kr210.urdf.xarco file and put them in the below relative joint location table

![alt text][image3]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

![DH Param Table][image1]

Create transofrmation matricies about each joint, generalized homogeneous transform between base_link and gripper using only the gripper pose.

    s = {alpha0:     0, a0:      0, d1:  0.75, 
         alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2 - pi/2,
         alpha2:     0, a2:   1.25, d3:     0,
         alpha3: -pi/2, a3: -0.054, d4:  1.50,
         alpha4:  pi/2, a4:      0, d5:     0,
         alpha5: -pi/2, a5:      0, d6:     0,
         alpha6:     0, a6:      0, d7: 0.303,         q7: 0}

    def dh_transform(q, alpha, a, d):
            return Matrix([[           cos(q),           -sin(q),           0,   a],
                   [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha *d],
                   [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                   [                0,                 0,           0,             1]])

    T0_1 = dh_transform(q1, alpha0, a0, d1)
    T1_2 = dh_transform(q2, alpha1, a1, d2)
    T2_3 = dh_transform(q3, alpha2, a2, d3)
    T3_4 = dh_transform(q4, alpha3, a3, d4)
    T4_5 = dh_transform(q5, alpha4, a4, d5)
    T5_6 = dh_transform(q6, alpha5, a5, d6)
    T6_G = dh_transform(q7, alpha6, a6, d7)

    T0_1 = T0_1.subs(s)
    T1_2 = T1_2.subs(s)
    T2_3 = T2_3.subs(s)
    T3_4 = T3_4.subs(s)
    T4_5 = T4_5.subs(s)
    T5_6 = T5_6.subs(s)
    T6_G = T6_G.subs(s)
    
    T0_2 = (T0_1 * T1_2)
    T0_3 = (T0_2 * T2_3)
    T0_4 = (T0_3 * T3_4)
    T0_5 = (T0_4 * T4_5)
    T0_6 = (T0_5 * T5_6)
    T0_G = simplify(T0_6 * T6_G)

    R_z = make_homogeneous(rot_z(pi), Matrix([[0],[0],[0]]))
    R_y = make_homogeneous(rot_y(-pi/2), Matrix([[0],[0],[0]]))
    R_corr = simplify(R_z * R_y)

    T0_total = simplify(T0_G * R_corr)


T0_Total is the end effector/gripper post

The above gives us the outcome of the end effector in the frame by using forward kinematics 

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Step 1 - Determine the DH Parameter table, see below

![DH Param Table][image1]

Step 2 - Determine the location of the WC relative to the base frame

![inverse-kinematics][image4]

The above image is from the Udacity Slack Channel 



### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  

First step is to define the DH param table (placeholder values)

            # Define DH param symbols
            d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8")
            a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7")
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols("alpha0:7")
            q1, q2, q3, q4, q5, q6, q7 = symbols("q1:8")
            
Then define the joint able symbols (again 0 values)


            # Joint angle symbols
            theta1, theta2, theta3, theta4, theta5, theta6 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0


As explained in section 2 I then added in the DH params table that was derived and created the transformation matrices. Now we can turn our attention to the end effector (gripper) position.

First step is to get the x, y and z position from inbound information (req)

            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])


