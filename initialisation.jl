using Polynomials

###########################
# Constants of the aircraft
###########################
const physical_data = physic_cst()
const aircraft = avion()


#######################
# for solver resolution 
#######################
const Tmax = 1.
const T0 = 0.
const dt = 0.01


###########################################
# functions to initialise state and control
###########################################
function create_initial_state(position, angle_deg,aircraft_physical_data)
    # Initiatz the variable
    X0 = zeros(Float64,11)
    # Position
    X0[1:3] = position
    # Orientation
    w,x,y,z = angles2quaternion(0,-angle_deg*pi/180,0)
    X0[7:10] .= w,x,y,z
    # Vitesse
    c_lift, aircraft, physical_data = aircraft_physical_data
    V2 = angle2vitesse(angle_deg,X0[3],aircraft.gross_mass,aircraft_physical_data)^2
    v0x = sqrt(V2) 
    v0y = 0
    v0z = 0
    X0[4:6] .= v0x,v0y,v0z
    # Masse
    X0[11] = aircraft.gross_mass
    return X0
end

# give the good thrust to keep the aircraft on the same level
function poussee(altitude, speed, mass, angle_deg,aircraft_physical_data)
    c_lift, aircraft, physical_data = aircraft_physical_data
    V = sum(speed.^ 2)
    rho = masse_volumique(altitude, pressure, physical_data.r,physical_data.T)
    coeff_aero = get_coeff(c_lift,angle_deg)

    return 0.5 * V * aircraft.Sw * rho * coeff_aero.Drag - sin(angle_deg*pi/180) * mass * physical_data.g[3]
end


##################
# Initialise state
################## 
position = [0,0,0]
Angle = 5.
aircraft_physical_data = (c_lift,aircraft, physical_data)
X0 = create_initial_state(position,Angle,aircraft_physical_data)


####################
# Initialise control
#################### 
poussee_init = poussee(X0[3],X0[4:6],X0[11],Angle,aircraft_physical_data)
poussee_echelon = 1.01 * poussee_init

angle_echelon = find_angle(poussee_echelon, X0[11],aircraft_physical_data)


# fonction pour faire évoluer la poussée(U[1])
function linear_poussee(t0,tf,poussee_init,poussee_echelon,l,h)
    return fit([t0,tf,t0+(tf-t0)*l],[poussee_init,poussee_echelon,poussee_echelon+(poussee_echelon-poussee_init)*h])
end

function evolve_thrust(t, poussee_init)
    if  1 < t < 10
        T = poussee_init + (t-1) * 10
    elseif  10 <= t < 19
        T = poussee_init - (t-19) * 10
    else 
        T = poussee_init
    end
    return T
end

# fonction définissant le controle
function control(t,mass,dry_mass)
    T = poussee_init
    U = @MArray [T,0,0,0]
    if  false#3 > t >= 1
        U = @MArray [poussee_echelon,0,0,0]
    end
    if mass <= dry_mass
        U[1] = 0
    end
    return U
end
