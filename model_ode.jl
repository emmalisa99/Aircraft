###############################################################################
##                        Fonctions pour lignes droites                      ##
###############################################################################

function f(dX,X,p,t=0)
    """
    dot X = f(X,U)
    modelling the physical behavior of the aircraft
    Input :
       X = [x1,x2,x3,  v1,v2,v3,  q1,q2,q3,q4, m] : position, speed, quaternion, masse
       U = [U1,  U2,U3,U4] : thrust and others forces for the rotation
    """
    
    # initialisation des paramètres
    c_lift, aircraft, physical_data, control, angle_function = p
    U = control(t,X[11],aircraft.dry_mass)
    
    # sans vairation de masse
    @views P_I2B = quaternion2matrix(X[7:10]...)
    @views i = P_I2B[:,1]
    assiette_rad = asin(max(-1,min(1,-P_I2B[3,1]))) 
    assiette = - assiette_rad * 180 / pi
    
    # # essai pour changement angle pitch/assiette
    # assiette = angle_function(t) 
    # X[7:10] .= angle2quaternion(assiette,[0,1,0])
    # @views P_I2B = quaternion2matrix(X[7:10]...)

    # # si vairation de masse
    # assiette,U[1] = dm2AngleThrust(X,(c_lift,aircraft,physical_data),5)
    # X[7:10] .= angles2quaternion(0,assiette_rad,0)
    # @views P_I2B = quaternion2matrix(X[7:10]...)

    coeff_aero = get_coeff(c_lift,assiette)
    C_L = coeff_aero.Lift
    C_D = coeff_aero.Drag

    # calcul des forces
    @views eta_a = masse_volumique(X[3], pressure, physical_data.r,physical_data.T) * aircraft.Sw * 0.5
    @views v_body = transpose(P_I2B) * X[4:6]
    norm2_v_body = sum(v_body .^ 2)
    Flift = eta_a .* norm2_v_body .* P_I2B * @SArray [0.,0.,C_L] 
    Fdrag = -eta_a .* norm2_v_body .* P_I2B * @SArray [C_D,0.,0.]
    
    # matrice pour le changement d'orientation
    @views M = @SArray([0 -U[2] -U[3] -U[4] ;
                       U[2] 0 U[4] -U[3] ;
                       U[3] -U[4] 0 U[2] ;
                       U[4] U[3] -U[2] 0] )  

    # résultat : dérivée de l'état
    @views dX[1:3] .= X[4:6]                                                         # variation of position
    @. @views dX[4:6] = (Flift + Fdrag + U[1]*i)/X[11] + physical_data.g             # variation of speed
    @views dX[7:10] .= 1/2 .* M * X[7:10]                                            # variation pf the orientation
    @views dX[11] = 0#-aircraft.kt * U[1]                                            # variation of fuel
    
    return dX
end


######################################################################
###                             Virage                              ##
######################################################################

# oriente l'avion (angle yaw seuleument) dans le sens de la vitesse
function find_yaw(V)
    yaw_rad = acos(V[1]/(sqrt(sum(V[1:3].^2))))
    return yaw_rad
end

# trouve quaternion avec mise à jour de l'angle yaw
function update_q_with_yaw(X)
    P_I2B = quaternion2matrix(X[7:10]...)
    assiette_rad = asin(max(-1,min(1,-P_I2B[3,1])))
    roll_rad = acos( max(-1,min(1,P_I2B[3,3]/cos(assiette_rad))))
    yaw_rad = find_yaw(X[4:6])
    return angles2quaternion(roll_rad,assiette_rad,yaw_rad)
end

# fonction du modèle
function f_turn(dX,X,p,t=0)  

    # initialisation des paramètres
    c_lift, aircraft, physical_data, control, angle_function = p
    U = control(t,X[11],aircraft.dry_mass)

    X[7:10] .= update_q_with_yaw(X)
    @views P_I2B = quaternion2matrix(X[7:10]...)

    assiette_rad = asin(max(-1,min(1,-P_I2B[3,1]))) 
    assiette = assiette_rad / pi * 180
    coeff_aero = get_coeff(c_lift,-assiette)
    C_L = coeff_aero.Lift
    C_D = coeff_aero.Drag

    yaw_rad = acos(max(-1,min(1,P_I2B[1,1]/cos(assiette_rad))))

    # changement du roll
    if  1 < t <= 10
        roll_rad =  -(4*t-4) * pi / 180 
    elseif  10 < t <= 20
        roll_rad =  -(4*10-4) * pi / 180
    elseif 20 < t <= 29
        roll_rad =  ((36/9)*t-(36*29/9)) * pi / 180
    else
        roll_rad = 0.
    end  

    # mise a jour de l'orientation avec le nouveau roll
    X[7:10] .= angles2quaternion(roll_rad,assiette_rad,yaw_rad)
    P_I2B = quaternion2matrix(X[7:10]...)
    @views i = P_I2B[:,1]
    # mise a jour de la poussée pour garantir équilibre vertical
    U[1] = find_poussee(X,P_I2B,(c_lift, aircraft, physical_data)) 

    # calcul des forces
    @views v_body = transpose(P_I2B) * X[4:6]
    norm2_v_body = sum(v_body .^ 2)
    @views eta_a = masse_volumique(X[3], pressure, physical_data.r,physical_data.T) * aircraft.Sw * 0.5
    Flift = eta_a .* norm2_v_body .* P_I2B *   @SArray [0.,0.,C_L] 
    Fdrag =  -eta_a .* norm2_v_body .* P_I2B  * @SArray [C_D,0.,0.]
    
    # matrice pour le changement d'orientation
    @views M = @SArray([0 -U[2] -U[3] -U[4] ;
    U[2] 0 U[4] -U[3] ;
    U[3] -U[4] 0 U[2] ;
    U[4] U[3] -U[2] 0] )  

    # résultat : dérivée de l'état
    @views dX[1:3] .= X[4:6]                                    
    @. @views dX[4:6] =  (Flift + Fdrag)/X[11] + physical_data.g  + U[1]/X[11] *  i  
    @views dX[7:10] .= 1/2 .* M * X[7:10]
    @views dX[11] = 0#-aircraft.kt * U[1]      

    return dX
end



###############################################################################
####               Obtenir les forces en fonction du modèle                  
###############################################################################

function forces(X,p,t)

    # initialisation des paramètres
    c_lift, aircraft, physical_data, control, angle_function = p
    U = control(t,X[11],aircraft.dry_mass)
    
    # sans vairation de masse
    @views P_I2B = quaternion2matrix(X[7:10]...)
    @views i = P_I2B[:,1]
    assiette_rad = asin(max(-1,min(1,-P_I2B[3,1]))) 
    assiette = - assiette_rad * 180 / pi

    # # si vairation de masse
    # assiette,U[1] = dm2AngleThrust(X,(c_lift,aircraft,physical_data),5)
    # X[7:10] .= angles2quaternion(0,assiette_rad,0)
    # @views P_I2B = quaternion2matrix(X[7:10]...)

    coeff_aero = get_coeff(c_lift,assiette)
    C_L = coeff_aero.Lift
    C_D = coeff_aero.Drag

    # calcul des forces
    @views eta_a = masse_volumique(X[3], pressure, physical_data.r,physical_data.T) * aircraft.Sw * 0.5
    @views v_body = transpose(P_I2B) * X[4:6]
    norm2_v_body = sum(v_body .^ 2)
    
    Flift = eta_a .* norm2_v_body .* P_I2B * @SArray [0.,0.,C_L] 
    Fdrag = -eta_a .* norm2_v_body .* P_I2B * @SArray [C_D,0.,0.]
    Thrust = U[1] .* i

    force = Forces(Flift,Fdrag,Thrust,X[1:3],X[11])

    return force
end
