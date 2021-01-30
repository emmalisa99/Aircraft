using Plots
using Roots


##################################################################################################
### Si l'équation des forces verticales est satisfaites, alors l'équation suivante doit être nulle
##################################################################################################

function relation_assiette_speed_thrust(assiette,thrust,mass,aircraft_physical_data)
    """relation liant assiette/pitch, vitesse et la poussée (considère roll=0, deux équations
    (dérivée de vitesse en i,k du repère avion) donnent cette relation)
    """ 
    c_lift, aircraft, physical_data = aircraft_physical_data
    coeff_aero = get_coeff(c_lift,assiette) 
    assiette_rad = assiette * pi / 180
    poids = - physical_data.g[3] * mass
    return -cos(assiette_rad) * poids * coeff_aero.Drag / coeff_aero.Lift - sin(assiette_rad) * poids + thrust
end
find_angle(thrust,mass,aircraft_physical_data) = fzero(assiette->relation_assiette_speed_thrust(assiette,thrust,mass,aircraft_physical_data), 10)


function angle2vitesse(assiette,altitude,masse,aircraft_physical_data)
    """
    Obj : donner la vitesse garantissant la dérivée de la vitesse nulle, en fonction de l'angle
    Remark : considère qu'il n'y a pas d'angle roll.
    """
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

function dm2angle(angle_deg,X,aircraft_physical_data)
    """
    Obj : équation de la dérivée de la vitesse (coordonnée verticale du repère avion - considère que roll = 0)
    """
    c_lift, vaircraft, physical_data = aircraft_physical_data
    coeff_aero = get_coeff(c_lift,angle_deg)
    V = sum(X[4:6] .^2)
    mass_vol = masse_volumique(X[3], pressure, physical_data.r,physical_data.T)
    angle_rad = angle_deg*pi/180
    return  mass_vol * aircraft.Sw * V * coeff_aero.Lift / 2 + cos(angle_rad)* X[11] * physical_data.g[3]  
end

# fonction pour retrouver l'angle selon la masse
find_dm2angle(X,aircraft_physical_data) = fzero(angle_deg->dm2angle(angle_deg,X,aircraft_physical_data), 10)

function dm2AngleThrust(X,aircraft_physical_data,angle_init)
    """
    Objectif :  trouver l'angle et la poussée pour garder l'équilibre (en fonction de la masse)
    """
    altitude = X[3] 
    speed = X[4:6]
    mass = X[11]

    angle_deg  = find_dm2angle(X,aircraft_physical_data)
    thrust = poussee(altitude,speed,mass,angle_deg,aircraft_physical_data)

    return angle_deg,thrust
end


####################################################################################
###                  fonctions pour trouver l'équilibre dans cas du virage
####################################################################################

# trouver la poussée qui permet de garder équilibre vertical
function find_poussee(X,P_I2B,aircraft_physical_data)

    # calcul des paramètres
    assiette_rad = asin(max(-1,min(1,-P_I2B[3,1])))
    assiette = assiette_rad * 180 / pi 

    mass = X[11] 
    V2 = sum(X[4:6].^2)
    c_lift, aircraft, physical_data = aircraft_physical_data
    mass_vol = masse_volumique(X[3], pressure, physical_data.r,physical_data.T)

    # calcul forces aérodynamiques
    coeff_aero = get_coeff(c_lift,-assiette) 
    poids = physical_data.g[3] * mass
    trainee = - P_I2B[3,1] * 1/2 * mass_vol * aircraft.Sw * V2 * coeff_aero.Drag
    portance = P_I2B[3,3] * 1/2 * mass_vol * aircraft.Sw * V2 * coeff_aero.Lift
    
    return -(trainee + portance + poids)/ P_I2B[3,1]
end