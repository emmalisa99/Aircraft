#include("debut.jl")
using StaticArrays
using Rotations

struct Forces
    Lift::SArray{Tuple{3},Float64,1,3}
    Drag::SArray{Tuple{3},Float64,1,3}
    Thrust::SArray{Tuple{3},Float64,1,3}
    Position::SArray{Tuple{3},Float64,1,3}
end

struct Coeff
    Lift::Float64
    Drag::Float64
end

angle2coeff = Dict()
angle2coeff[-4] = Coeff(-0.09,0.010)
angle2coeff[-2] = Coeff(0.05,0.009)
angle2coeff[0] = Coeff(0.2,0.010)
angle2coeff[2] = Coeff(0.36,0.015)
angle2coeff[4] = Coeff(0.51,0.022)
angle2coeff[6] = Coeff(0.66,0.033)
angle2coeff[8] = Coeff(0.80,0.045)
angle2coeff[10] = Coeff(0.94,0.062)

function f(dX,X,p,t=0)
    """
    dot X = f(X,U)
    modelling the physical behavior of the aircraft
    Input :
       X = [x1,x2,x3,  v1,v2,v3,  q1,q2,q3,q4, m] : position, speed, quaternion, masse
       U = [U1,  U2,U3,U4] : thrust and others forces for the rotation
    """
    physical_data, aircraft, Ut = p
    U = Ut(t,X)
    q = UnitQuaternion(X[7:10]...)

    @views P_I2B = @SArray([2*(X[7]^2+X[8]^2)-1       2*(X[8]*X[9]-X[7]*X[10])  2*(X[8]*X[10]+X[9]*X[7]);
    2*(X[8]*X[9]+X[7]*X[10])  2*(X[7]^2+X[9]^2)-1       2*(X[9]*X[10]-X[8]*X[7]);
    2*(X[8]*X[10]-X[9]*X[7])  2*(X[9]*X[10]+X[8]*X[7])  2*(X[7]^2+X[10]^2)-1])

    @views i = P_I2B[:,1]

    @views M = @SArray([0 -U[2] -U[3] -U[4] ;
                        U[2] 0 U[4] -U[3] ;
                        U[3] -U[4] 0 U[2] ;
                        U[4] U[3] -U[2] 0] )        # matrix for quaternion

    @views v_body = transpose(P_I2B) * X[4:6] # Peut faire uniquement la transposé car changement de base entre repères orthonormés
    norm2_v_body = sum(v_body .^ 2)

    # @views alpha = asin(v_body[3]/ sqrt(norm2_v_body))
    # @views beta = asin(v_body[2]/ sqrt(norm2_v_body))
    # ca = cos(alpha)
    # cb = cos(beta)
    # sa = sin(alpha)
    # sb = cos(beta)
    # @views P_B2W = @SArray([ ca*cb -ca*sb -sa;
    #                 sb cb 0;
    #                 sa*cb -sa*sb ca])                            # "passage" matrix to wind coordinate to aircraft coordinate
    # C_D = 0.031#aircraft_cst.C_D0  + aircraft_cst.C_D_alpha2 * alpha^2
    # C_C = aircraft_cst.C_C_beta * beta
    # C_L = 0.31 #aircraft_cst.C_L_alpha * alpha

    alpha = atan(v_body[3]/sqrt(v_body[1]^2+v_body[2]^2))
    alpha = Int(floor(alpha/2))*2
    C_L,C_D = angle2coeff[alpha].Lift,angle2coeff[alpha].Drag 

    @views eta_a = rho(X[3], pressure, physical_data.r,physical_data.T) * aircraft_cst.Sw * 0.5
    @views norm2_speed = (X[4]^2+X[5]^2+X[6]^2)
    Flift = eta_a .* norm2_v_body .* P_I2B *   @SArray [0.,0.,C_L]  # aerodynamical forces
    Fdrag =  -eta_a .* norm2_v_body .* P_I2B  * @SArray [C_D,0.,0.]

    @views dX[1:3] .= X[4:6]                                    # dot x = v
    @. @views dX[4:6] =  (Flift + Fdrag)/X[11] + physical_data.g  + U[1]/X[11] * (aircraft.kt*X[4:6] + i)
    @views dX[7:10] .= Rotations.kinematics(q,U[2:4])# 1/2 .* M * X[7:10] ##1 # .*q en utilisant @views q = X[7:10] / pour 1/2 .* avec .= au début
    @views dX[11] = -aircraft.kt * U[1]                        # dot m = -kt * trhust : variation of fuel
    return dX
end


function forces(X,p,t)

    physical_data, aircraft, Ut = p
    U = Ut(t,X)

    @views P_I2B = @SArray([2*(X[7]^2+X[8]^2)-1       2*(X[8]*X[9]-X[7]*X[10])  2*(X[8]*X[10]+X[9]*X[7]);
    2*(X[8]*X[9]+X[7]*X[10])  2*(X[7]^2+X[9]^2)-1       2*(X[9]*X[10]-X[8]*X[7]);
    2*(X[8]*X[10]-X[9]*X[7])  2*(X[9]*X[10]+X[8]*X[7])  2*(X[7]^2+X[10]^2)-1])

    @views i = P_I2B[:,1]

    @views v_body = transpose(P_I2B) * X[4:6] # Peut faire uniquement la transposé car changement de base entre repères orthonormés
    norm2_v_body = sum(v_body .^ 2)
                     
    C_D = 0.031#aircraft_cst.C_D0  + aircraft_cst.C_D_alpha2 * alpha^2
    C_L = 0.31 #aircraft_cst.C_L_alpha * alpha
    @views eta_a = rho(X[3], pressure, physical_data.r,physical_data.T) * aircraft_cst.Sw * 0.5
    @views norm2_speed = (X[4]^2+X[5]^2+X[6]^2)
    
    Flift = eta_a .* norm2_v_body .* P_I2B *   @SArray [0.,0.,C_L]  
    Fdrag =  -eta_a .* norm2_v_body .* P_I2B  * @SArray [C_D,0.,0.]
    Thrust = U[1] .* i

    force = Forces(Flift,Fdrag,Thrust,X[1:3])

    return force
end