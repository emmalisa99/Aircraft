using Rotations



function forces(X,p,t)

    physical_data, aircraft, Ut = p
    U = Ut(t,X)

    @views P_I2B = @SArray([2*(X[7]^2+X[8]^2)-1       2*(X[8]*X[9]-X[7]*X[10])  2*(X[8]*X[10]+X[9]*X[7]);
    2*(X[8]*X[9]+X[7]*X[10])  2*(X[7]^2+X[9]^2)-1       2*(X[9]*X[10]-X[8]*X[7]);
    2*(X[8]*X[10]-X[9]*X[7])  2*(X[9]*X[10]+X[8]*X[7])  2*(X[7]^2+X[10]^2)-1])

    @views i = P_I2B[:,1]

    P_I2B = UnitQuaternion(X[7:10]...)
    @views i = P_I2B[:,1]

    @views v_body = P_I2B * X[4:6] # Peut faire uniquement la transposé car changement de base entre repères orthonormés
    norm2_v_body = sum(v_body .^ 2)
                     
    assiette = 2 * asin(sqrt(X[8]^2+X[9]^2+X[10]^2)) * 180 / pi
    coeff_aero = get_coeff(c_lift,assiette)
    C_L = coeff_aero.Lift
    C_D = coeff_aero.Drag

    @views eta_a = masse_volumique(X[3], pressure, physical_data.r,physical_data.T) * aircraft.Sw * 0.5
    @views norm2_speed = (X[4]^2+X[5]^2+X[6]^2)
    
    Flift = eta_a .* norm2_v_body .* P_I2B *   @SArray [0.,0.,C_L]  
    Fdrag =  -eta_a .* norm2_v_body .* P_I2B  * @SArray [C_D,0.,0.]
    Thrust = U[1] .* i

    force = Forces(Flift,Fdrag,Thrust,X[1:3],X[11])

    return force
end



function f(dX,X,p,t=0)
    """
    dot X = f(X,U)
    modelling the physical behavior of the aircraft
    Input :
       X = [x1,x2,x3,  v1,v2,v3,  q1,q2,q3,q4, m] : position, speed, quaternion, masse
       U = [U1,  U2,U3,U4] : thrust and others forces for the rotation
    """
  
    c_lift, aircraft, physical_data, control, angle_function = p
    U = control(t,X[11],aircraft.dry_mass)

    is_equilibrate = is_stable(X,U[1],(c_lift, aircraft, physical_data))
    
    if is_equilibrate
        # sans vairation de masse
        assiette = 2 * asin(sqrt(X[8]^2+X[9]^2+X[10]^2)) * 180 / pi
        
        # if is_equilibrate && assiette != 5.
        #     println("temps : ",t)
        #     println("-------------- Ici, on est a l'équilibre----------------")
        # end
        coeff_aero = get_coeff(c_lift,assiette)
        C_L = coeff_aero.Lift
        C_D = coeff_aero.Drag
    else 
        # essai pour echelon
        assiette = angle_function(t) #2 * asin(sqrt(X[8]^2+X[9]^2+X[10]^2)) * 180 / pi
        # if abs(assiette - angle_echelon) < 1e-2
        #     assiette = angle_echelon
        # end 
        X[7:10] .= angle2quaternion(assiette,[0,1,0])
        coeff_aero = get_coeff(c_lift,assiette)
        C_L = coeff_aero.Lift
        C_D = coeff_aero.Drag
    end

    # # si vairation de masse
    # assiette,U[1] = dm2AngleThrust(X,(c_lift,aircraft,physical_data),5)
    # X[7:10] .= angle2quaternion(assiette,[0,1,0])
    # coeff_aero = get_coeff(c_lift,assiette)
    # C_L = coeff_aero.Lift
    # C_D = coeff_aero.Drag
    
    @views P_I2B = UnitQuaternion(X[7:10]...)
    @views i = P_I2B[1,:]

    @views M = @SArray([0 -U[2] -U[3] -U[4] ;
                        U[2] 0 U[4] -U[3] ;
                        U[3] -U[4] 0 U[2] ;
                        U[4] U[3] -U[2] 0] )       

    @views v_body = P_I2B * X[4:6]#transpose(X[4:6]) * transpose(P_I2B) # Peut faire uniquement la transposé car changement de base entre repères orthonormés
    norm2_v_body = sum(v_body .^ 2)

    @views eta_a = masse_volumique(X[3], pressure, physical_data.r,physical_data.T) * aircraft.Sw * 0.5
    @views norm2_speed = (X[4]^2+X[5]^2+X[6]^2)
    Flift = eta_a .* norm2_v_body .* transpose(P_I2B) *   @SArray [0.,0.,C_L] 
    Fdrag =  -eta_a .* norm2_v_body .* transpose(P_I2B)  * @SArray [C_D,0.,0.]
    
    @views dX[1:3] .= X[4:6]                                    # dot x = v
    @. @views dX[4:6] =  (Flift + Fdrag)/X[11] + physical_data.g  + U[1]/X[11] *  i  #(aircraft.kt*X[4:6] +) 
    @views dX[7:10] .= Rotations.kinematics(P_I2B,U[2:4])# 1/2 .* M * X[7:10] 
    @views dX[11] = 0#-aircraft.kt * U[1]                        # dot m = -kt * trhust : variation of fuel
    
    if is_equilibrate && assiette != 5
        println(" Dérivée = ", dX[4:6])
    end
    if assiette > 5.112 && assiette < 5.114#false
        println("t = ",t, ",        Vitesse = ",sqrt(sum(X[4:6].^2)), ",", X[4:6], " Dérivée = ", dX[4:6])
        println("Assiette : ",assiette)
        #println("Pousse :", U[1])
    end
    if abs(X[6]) < 1e-5 && assiette != 5.
        println("Angle à Vz = 0 : ", assiette, "    (module = ", sqrt(sum(X[4:6].^2)),")") 
    end

    return dX
end




