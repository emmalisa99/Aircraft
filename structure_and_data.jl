##################################
#####     Physical data      #####
##################################

struct physic_cst
    g::SVector{3, Float64}      #Gravity vector
    r::Float64  #Air perfect gas specific constant
    T::Float64  #Temperature
    physic_cst(g = @SArray([0,0,-9.81]), r = 287., T = 288.15) = new(g,r,T)
end

function pressure(height)
    p = 1013.25 * (1 - 0.0065*height * (1.0/288.15))^(5.255)
    return p
end

function masse_volumique(height, pressure, r, T)
    return 100 * pressure(height) * (1.0 / (r * T))
end

struct Forces
    Lift::SArray{Tuple{3},Float64,1,3}
    Drag::SArray{Tuple{3},Float64,1,3}
    Thrust::SArray{Tuple{3},Float64,1,3}
    Position::SArray{Tuple{3},Float64,1,3}
    Mass::Float64
end


##################################
#####   Aircraft Structure   #####
##################################

struct avion
    dry_mass::Float64   #Mass without fuel
    gross_mass::Float64
    kt::Float64         #Constant linked to the type of aircraft
    AR::Float64         # Aspect ratio
    Sw::Float64         # Wing area
    #max_thrust::Float64 #U1 maximum
    #max_amplitude_anguar_control::Float64   #U2,U3,U4 maximum
    #length::Float64
    #width::Float64
    #thickness::Float64

    avion(dry_mass = 779., gross_mass=1111, kt=0.0001,AR=7.4,Sw=16.2) = new(dry_mass , gross_mass, kt, AR, Sw)
end


