include("debut.jl")

function f(X,Ut,t=0)  
    """
    dot X = f(X,U)
    modelling the physical behavior of the aircraft
    Input : 
       X = [x1,x2,x3,  v1,v2,v3,  q1,q2,q3,q4, m] : position, speed, quaternion, masse
       U = [U1,  U2,U3,U4] : thrust and others forces for the rotation
    """

    U = Ut(t)

    i =  SA[2*(X[7]^2+X[8]^2)-1 , 
            2*(X[8]*X[9]+X[7]*X[10]) ,
            2*(X[8]*X[10]-X[7]*X[9]) ]               # effect of the rotation on speed
    
    M = [0 -U[2] -U[3] -U[4] ;
        U[2] 0 U[4] -U[3] ; 
        U[3] -U[4] 0 U[2] ; 
        U[4] U[3] -U[2] 0]                            # matrix for quaternion 

    P_I2B = [2*(X[7]^2+X[8]^2)-1       2*(X[8]*X[9]-X[7]*X[10])  2*(X[8]*X[10]+X[9]*X[7]);
             2*(X[8]*X[9]+X[7]*X[10])  2*(X[7]^2+X[9]^2)-1       2*(X[9]*X[10]-X[8]*X[7]);
             2*(X[8]*X[10]-X[9]*X[7])  2*(X[9]*X[10]+X[8]*X[7])  2*(X[7]^2+X[10]^2)-1]

    v_body = inv(P_I2B) * X[4:6]
    norm2_v_body = (v_body[1]^2 + v_body[2]^2 + v_body[3]^2) 
    norminf_v_body = max(v_body[1], v_body[2], v_body[3])

    # alpha = asin(-X[6]/norminf_v_body) #
    # beta = asin(-X[5]/norminf_v_body)  #

    alpha = asin(-v_body[3]/ sqrt( norm2_v_body))
    beta = asin(-v_body[2]/ sqrt(norm2_v_body))
    ca = cos(alpha)
    cb = cos(beta)
    sa = sin(alpha)
    sb = cos(beta)
    P_B2W = [ ca*cb -ca*sb sa;
            sb cb 0;
            -sa*cb sa*sb ca]                             # "passage" matrix to wind coordinate to aircraft coordinate 
    C_D = aircraft_cst.C_D0  + aircraft_cst.C_D_alpha2 * alpha^2
    C_C = aircraft_cst.C_C_beta * beta
    C_L = aircraft_cst.C_L_alpha * alpha
    Cst_DCL = SA[-C_D ; -C_C ; C_L ] 
    eta_a = rho(X[3], pressure, physical_data.r,physical_data.T) * aircraft_cst.Sw * 0.5
    norm2_speed = (X[4]^2+X[5]^2+X[6]^2)
    Fa = eta_a * norm2_speed * P_I2B * P_B2W * Cst_DCL  # aerodynamical forces
    
    Xp = Vector(undef, 11)
    Xp[1:3] = X[4:6]                                    # dot x = v
    Xp[4:6] =  physical_data.g + U[1]/X[11] * (aircraft.kt*X[4:6] + i) + Fa/X[11] 
    Xp[7:10] = 1/2 * M * X[7:10]
    Xp[11] = -aircraft.kt * U[1]                        # dot m = -kt * trhust : variation of fuel

    return Xp
end


 
