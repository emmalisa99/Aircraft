using Plots
using Roots


##########################################
## Pour savoir si on a un état d'équilibre
##########################################
function is_stable(X,thrust,aircraft_physical_data)
    c_lift,aircraft,physical_data = aircraft_physical_data
    
    w,x,y,z = X[7:10]
    assiette_rad = 2 * asin(sqrt(x^2+y^2+z^2)) 
    assiette_deg = assiette_rad * 180 / pi
    coeff_aero = get_coeff(c_lift,assiette_deg)

    poids  = - X[11] * physical_data.g[3]
    V2 =  sum(X[4:6].^2)  
    etha_a = masse_volumique(X[3], pressure, physical_data.r,physical_data.T) * aircraft.Sw * 0.5

    eq_horizontal = abs(etha_a * V2 * coeff_aero.Lift - cos(assiette_rad) * poids) < 1e-11
    eq_vertical = abs(etha_a * V2 * coeff_aero.Drag - thrust + sin(assiette_rad) * poids) < 1e-11
    return eq_horizontal && eq_vertical
end


##################################################################################################
### Si l'équation des forces verticales est satisfaites, alors l'équation suivante doit être nulle
##################################################################################################
function relation_assiette_speed_thrust(assiette,thrust,mass,aircraft_physical_data)
    c_lift, aircraft, physical_data = aircraft_physical_data
    coeff_aero = get_coeff(c_lift,assiette) 
    assiette_rad = assiette * pi / 180
    poids = - physical_data.g[3] * mass
    return -cos(assiette_rad) * poids * coeff_aero.Drag / coeff_aero.Lift - sin(assiette_rad) * poids + thrust
end

find_angle(thrust,mass,aircraft_physical_data) = fzero(assiette->relation_assiette_speed_thrust(assiette,thrust,mass,aircraft_physical_data), 10)

function angle2vitesse(assiette,altitude,masse,aircraft_physical_data)
    c_lift, aircraft, physical_data = aircraft_physical_data
    coeff_aero = get_coeff(c_lift,assiette) 
    assiette = assiette * pi/180
    mass_vol= masse_volumique(altitude, pressure, physical_data.r,physical_data.T)
    return sqrt(-2 * cos(assiette) * masse * physical_data.g[3] /  (mass_vol * aircraft.Sw * coeff_aero.Lift))
end

function test(angle_voulu,altitude,mass,aircraft_physical_data)
    c_lift, aircraft, physical_data = aircraft_physical_data
    v0x = angle2vitesse(angle_voulu,altitude,mass,aircraft_physical_data)
    thrust = poussee(altitude, [v0x,0,0], mass, angle_voulu,aircraft_physical_data) # poussee_init # 2800
    
    # to create a plot of assiette in function of thrust at a certain speed
    list_assiettes = collect(-5.:10.)
    y = zeros(Float64,size(list_assiettes)[1])
    for i=1:size(list_assiettes)[1]
        y[i] = relation_assiette_speed_thrust(list_assiettes[i],thrust,mass,aircraft_physical_data)
    end

    #results
    println("Pour le premier résultat, on attend : 5°")
    println("Si traction T = ", thrust, ", alors l'angle est : ",find_angle(thrust,mass,aircraft_physical_data))
    println("Si traction T = 2800, alors l'angle est : ",find_angle(2800,mass,aircraft_physical_data))

    plot(list_assiettes,y)
end 


#########################################
## dans le cadre d'une variation de masse
#########################################

# function dm2angle(angle_deg,mass,aircraft_physical_data,angle_init)
#     c_lift, vaircraft, physical_data = aircraft_physical_data
#     angle_rad = angle_deg*pi/180
#     C_L_0 = c_lift[2] * angle_init + c_lift[1]
#     return cos(angle_rad)*mass / (c_lift[2] * angle_deg + c_lift[1]) - cos(angle_rad) * aircraft.gross_mass / C_L_0
# end

function dm2angle(angle_deg,X,aircraft_physical_data)
    c_lift, vaircraft, physical_data = aircraft_physical_data
    coeff_aero = get_coeff(c_lift,angle_deg)
    V = sum(X[4:6] .^2)
    mass_vol = masse_volumique(X[3], pressure, physical_data.r,physical_data.T)
    angle_rad = angle_deg*pi/180
    return  mass_vol * aircraft.Sw * V * coeff_aero.Lift / 2 + cos(angle_rad)* X[11] * physical_data.g[3]  
end

# find_dm2angle(mass,aircraft_physical_data,angle_init) = fzero(angle_deg->dm2angle(angle_deg,mass,aircraft_physical_data,angle_init), 0)

# function dm2AngleThrust(X,aircraft_physical_data,angle_init)
#     altitude = X[3] 
#     speed = X[4:6]
#     mass = X[11]

#     angle_deg  = find_dm2angle(mass,aircraft_physical_data,angle_init)
#     thrust = poussee(altitude,speed,mass,angle_deg,aircraft_physical_data)#0.5 * mass_vol * V2 * coeff_aero.Drag * aircraft.Sw - sin(angle_deg*pi/180) * mass * physical_data.g[3]
   
#     return angle_deg,thrust
# end

find_dm2angle(X,aircraft_physical_data) = fzero(angle_deg->dm2angle(angle_deg,X,aircraft_physical_data), 10)

function dm2AngleThrust(X,aircraft_physical_data,angle_init)
    altitude = X[3] 
    speed = X[4:6]
    mass = X[11]

    angle_deg  = find_dm2angle(X,aircraft_physical_data)
    thrust = poussee(altitude,speed,mass,angle_deg,aircraft_physical_data)#0.5 * mass_vol * V2 * coeff_aero.Drag * aircraft.Sw - sin(angle_deg*pi/180) * mass * physical_data.g[3]
   
    return angle_deg,thrust
end
