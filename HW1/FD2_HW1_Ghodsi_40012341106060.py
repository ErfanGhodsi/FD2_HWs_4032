# Name: Erfan Ghodsi
# Num: 40012341106060

# Importing necessary libraries
import numpy as np
import math as mth

def question_1():
    # Inputs and constants for Question 1
    print("\n-- Question 1 --")
    Velocity = float(input("Velocity (meters/second)= "))  # Speed
    phi = float(input("phi (degrees)= "))  # Bank angle
    phi_rad = (3.14 / 180) * phi  # Convert angle to radians
    theta_1 = 0  # Initial value of theta
    psi = 0  # Initial value of psi
    g = 9.81  # Gravitational acceleration

    # Calculating P1, Q1, and R1 for coordinated turn
    P1 = -Velocity * mth.sin(theta_1)
    Q1 = Velocity * mth.cos(theta_1) * mth.sin(phi_rad)
    R1 = Velocity * mth.cos(theta_1) * mth.cos(phi_rad)

    # Angular velocity vector in the body frame
    Angular_velocity_in_body_frame = np.array([[P1], [Q1], [R1]])

    # Display angular velocity vector in the body frame
    print("Angular_velocity_in_body_frame:")
    print(Angular_velocity_in_body_frame)

    # Transformation matrix (from inertial to body frame)
    C_bi = np.array([
        [mth.cos(psi) * mth.cos(theta_1), mth.cos(theta_1) * mth.sin(psi), -mth.sin(theta_1)],
        [mth.cos(psi) * mth.sin(phi_rad) * mth.sin(theta_1) - mth.cos(phi_rad) * mth.sin(psi),
         mth.cos(phi_rad) * mth.cos(psi) + mth.sin(phi_rad) * mth.sin(theta_1) * mth.sin(psi),
         mth.cos(theta_1) * mth.sin(phi_rad)],
        [mth.sin(phi_rad) * mth.sin(psi) + mth.cos(phi_rad) * mth.cos(psi) * mth.sin(theta_1),
         mth.cos(phi_rad) * mth.sin(theta_1) * mth.sin(psi) - mth.cos(psi) * mth.sin(phi_rad),
         mth.cos(phi_rad) * mth.cos(theta_1)]
    ])

    # Calculate the transformation matrix (from body frame to inertial frame)
    C_ib = C_bi.T

    # Angular velocity vector in the inertial frame
    Angular_velocity_in_inertial_frame = np.dot(C_ib, Angular_velocity_in_body_frame)

    # Display angular velocity vector in the inertial frame
    print("Angular_velocity_in_inertial_frame:")
    print(Angular_velocity_in_inertial_frame)

def question_2():
    # Inputs and constants for Question 2
    print("\n-- Question 2 --")
    P = float(input("P (Roll rate in rad/s)= "))  # Roll rate in radians per second
    Q = float(input("Q (Pitch rate in rad/s)= "))  # Pitch rate in radians per second
    R = float(input("R (Yaw rate in rad/s)= "))  # Yaw rate in radians per second
    psi_dot = float(input("psi_dot (Yaw rate derivative in rad/s)= "))  # Derivative of yaw rate

    # Creating the angular velocity vector (P, Q, R)
    angular_velocity = np.array([[P], [Q], [R]])

    # Display the angular velocity vector
    print("Angular_velocity:")
    print(angular_velocity)

    # Calculating the Euler angles theta_1, phi_1, and psi_1
    theta_1 = mth.asin(-P / psi_dot)  # Calculation of theta (pitch angle)
    phi_1 = mth.asin(Q / (psi_dot * mth.cos(theta_1)))  # Calculation of phi (roll angle)
    psi_1 = -P / mth.sin(theta_1)  # Calculation of psi (yaw angle)

    # Display the Euler angles
    print("Euler_angles:")
    print(f"theta: {theta_1}")
    print(f"phi: {phi_1}")
    print(f"psi: {psi_1}")

def question_3():
    # Inputs and constants for Question 3
    print("\n-- Question 3 --")
    C_bi = np.array([
        [float(input("The element at (1,1)[0,0]: ")), float(input("The element at (1,2)[0,1]: ")), float(input("The element at (1,3)[0,2]: "))],
        [float(input("The element at (2,1)[1,0]: ")), float(input("The element at (2,2)[1,1]: ")), float(input("The element at (2,3)[1,2]: "))],
        [float(input("The element at (3,1)[2,0]: ")), float(input("The element at (3,2)[2,1]: ")), float(input("The element at (3,3)[2,2]: "))]
    ])

    # Check if matrix is a transformation matrix
    det_C_bi = np.linalg.det(C_bi)
    print("det_C_bi= ", det_C_bi)

    if abs(det_C_bi - 1) < 1e-6:
        print("The input matrix is a valid transformation matrix.")

        # Calculation of Euler angles: theta, psi, phi
        theta = mth.asin(-C_bi[0, 2])
        cos_theta = mth.cos(theta)

        if abs(cos_theta) > 1e-6:
            psi = mth.asin(C_bi[0, 1] / cos_theta)
            phi = mth.asin(C_bi[1, 2] / cos_theta)

            print("Euler_angles:")
            print(f"theta: {theta}")
            print(f"psi: {psi}")
            print(f"phi: {phi}")

            # Quaternion vector calculation
            a = (mth.cos(phi / 2) * mth.cos(theta / 2) * mth.cos(psi / 2)) - (mth.sin(phi / 2) * mth.sin(theta / 2) * mth.sin(psi / 2))
            b = (mth.sin(phi / 2) * mth.cos(theta / 2) * mth.cos(psi / 2)) + (mth.cos(phi / 2) * mth.sin(theta / 2) * mth.sin(psi / 2))
            c = (mth.cos(phi / 2) * mth.sin(theta / 2) * mth.cos(psi / 2)) - (mth.sin(phi / 2) * mth.cos(theta / 2) * mth.sin(psi / 2))
            d = (mth.cos(phi / 2) * mth.cos(theta / 2) * mth.sin(psi / 2)) - (mth.sin(phi / 2) * mth.sin(theta / 2) * mth.cos(psi / 2))

            Quaternion_vector = np.array([[a], [b], [c], [d]])
            print("Quaternion_vector:")
            print(Quaternion_vector)
        else:
            print("Singularity encountered during Euler angles calculation.")
    else:
        print("The input matrix is not a valid transformation matrix.")

# Menu to execute each question separately
while True:
    print("\nSelect a question to execute:")
    print("1. Question 1")
    print("2. Question 2")
    print("3. Question 3")
    print("4. Exit")
    
    choice = input("Enter your choice: ")
    
    if choice == "1":
        question_1()
    elif choice == "2":
        question_2()
    elif choice == "3":
        question_3()
    elif choice == "4":
        print("Exiting...")
        break
    else:
        print("Invalid choice. Please select again.")