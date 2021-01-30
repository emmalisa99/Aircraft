"""
Cette librairie a pour but de donner la représensation du quaternion
 à partir de trois angles (pitch,roll,yaw) ou d'une matrice de rotation,
 ou l'inverse.
"""

function angles2quaternion(roll,pitch,yaw)
    cy = cos(yaw * 0.5);
    sy = sin(yaw * 0.5);
    cp = cos(pitch * 0.5);
    sp = sin(pitch * 0.5);
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);

    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;

    return w,x,y,z
end

function quaternion2angles(w,x,y,z)
roll = atan( 2*(w*x+y*z) / (w^2-x^2-y^2+z^2))
pitch = asin( 2*(w*y-x*z))
yaw =  atan( 2*(w*z+y*x) / (w^2+x^2-y^2-z^2))
return roll,pitch,yaw
end


function angles2matrix(roll,pitch,yaw) #check
    @SArray [cos(pitch)*cos(yaw) -cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw) sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw);
    cos(pitch)*sin(yaw)  cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw) -sin(roll)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw);
    -sin(pitch) sin(roll)*cos(pitch) cos(roll)*cos(pitch)]
end

# function matrix2quaternion(P)
#     abs_q0 = sqrt(max(0,(1+P[1,1]+P[2,2]+P[3,3]/4)))
#     abs_q1 = sqrt(max(0,(1+P[1,1]-P[2,2]-P[3,3]/4)))
#     abs_q2 = sqrt(max(0,(1-P[1,1]+P[2,2]-P[3,3]/4)))
#     abs_q3 = sqrt(max(0,(1-P[1,1]-P[2,2]+P[3,3]/4)))

#     max_q = max(abs_q0,abs_q1,abs_q2,abs_q3)
#     if max_q == abs_q0
#         q_0 = abs_q0
#         q_1 = (P[3,2] - P[2,3])/(4*q_0)
#         q_2 = (P[1,3] - P[3,1])/(4*q_0)
#         q_3 = (P[2,1] - P[1,2])/(4*q_0)
#     elseif max_q == abs_q1
#         q_1 = abs_q1
#         q_0 = (P[3,2] - P[2,3])/(4*q_1)
#         q_2 = (P[1,2] - P[2,1])/(4*q_1)
#         q_3 = (P[1,3] - P[3,1])/(4*q_1)
#     elseif max_q == abs_q2
#         q_2 = abs_q2
#         q_0 = (P[1,3] - P[3,1])/(4*q_2)
#         q_1 = (P[1,2] - P[2,1])/(4*q_2)
#         q_3 = (P[2,3] - P[3,2])/(4*q_2)
#     else 
#         q_3 = abs_q3
#         q_0 = (P[1,2] - P[2,1])/(4*q_3)
#         q_1 = (P[1,3] - P[3,1])/(4*q_3)
#         q_2 = (P[2,3] - P[3,2])/(4*q_3)
#     end
#     return q_0,q_1,q_2,q_3
#end

function quaternion2matrix(w,x,y,z)
    P = [ w^2+x^2-y^2-z^2   2*x*y-2*w*z   2*x*z+2*w*y;
        2*x*y+2*w*z        w^2-x^2+y^2-z^2    2*y*z-2*w*x;
        2*x*z-2*w*y       2*y*z+2*w*x     w^2-x^2-y^2+z^2
    ]
    return P
end
