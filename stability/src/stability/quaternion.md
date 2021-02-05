# This is a summary of [Quaternion Physics](https://drive.google.com/file/d/12m8hsQJ-EKk8vDQYVwyyOT0U7RaDDAJw/view?usp=sharing)
## Anything with the subscript _vec represents a vector in R^3
i ** 2 = j ** 2 = k ** 2 = ijk = -1  # fundamental equation  
q = [q_0, q_1, q_2, q_3] = [q_0, q_vec]  # quaternion  
q_bar = [q_0, -q_vec]  # conjugate quaternion, which represents the inverse rotation  
|q| = 1  # This is required for q to represent a valid rotation  

## Applying a Quaternion Rotation to a Vector in R^3
x_vec = [x_1, x_2, x_3]  # vector in R^3 (in fixed reference frame)  
x = [0, x_1, x_2, x_3] = [0, x_vec]  # quaternion version of x_vec  
x' = q_bar * x * q = [0, x_vec']  # applying the rotation associated with q on x_vec gives x_vec'  
x = q * x' * q_bar  # This is just the inverse rotation which gets us back to x_vec  

## Rotational Velocity
w_vec = [w_1, w_2, w_3]  # angular velocity (in fixed reference frame)  
q_dot = (d/dt) q  # quaternion velocity  
w = [0, w_vec] = 2 * q_dot * q_bar  # quaternion version of w_vec  
w' = [0, w_vec'] = q_bar * w * q = 2 * q_bar * q_dot  # angular velocity in rotated reference frame  
q_dot = (1/2) * w * q = (1/2) * q * w'  

## Matrix Product Notation
            [-q_1,  q_0, -q_3,  q_2]  
w_vec = 2 * [-q_2,  q_3,  q_0, -q_1] * q_dot = 2 * E * q_dot = -2 * E_dot * q   
            [-q_3, -q_2,  q_1,  q_0]  
             [-q_1,  q_0,  q_3, -q_2]  
w_vec' = 2 * [-q_2, -q_3,  q_0,  q_1] * q_dot = 2 * G * q_dot = -2 * G_dot * q  
             [-q_3,  q_2, -q_1,  q_0]  
q_dot = (1/2) * E^T * w_vec = (1/2) * G^T * w_vec'  
E * q = G * q = 0_vec = [0, 0, 0]  

## Rotation Matrix
w_vec = R * w_vec'  # R is the rotation matrix equivalent of q  
R = E * G^T  
R^-1 = R^T = G * E^T  

## Equations of motion
Lagrangian = (1/2) * w_vec'^T * J * w_vec'  
State variables, are q = [q_0, q_1, q_2, q_3] with constraint that q^T * q = 1  
Applied torque in body reference frame is T_vec'  
The equations of motion are then:  
w_vec'_dot = J^-1 * T_vec' - J^-1 * (w_vec' x (J * w_vec'))  
q_dot = (1/2) G^T * w_vec'  
