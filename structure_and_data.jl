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

    #avion(dry_mass = 779., gross_mass=1111, kt=0.0001, max_thrust=1e3, max_amplitude_angular_control=10., length=4., width=1.5, thickness=0.7) = new(dry_mass , gross_mass, kt, max_thrust, max_amplitude_angular_control, length, width, thickness)
    avion(dry_mass = 779., gross_mass=1111, kt=0.0001,AR=7.4,Sw=16.2) = new(dry_mass , gross_mass, kt, AR, Sw)
end



#To improve : add C_D_alpha2 and AR (problems with connections between structures)
# struct MiniBee_cst#{T} => C. mettre T à la place de Float64
#     Cl::Float64     #Lift coefficient
#     Cd::Float64     #Drag coefficient
#     Ct::Float64

#     Sw::Float64     #Wing area
#     AR::Float64              #
#     C_D_alpha2::Float64
#     Sf::Float64     #Frontol area

#     C_L_alpha::Float64
#     C_D0::Float64
#     e_os::Float64
#     C_C_beta::Float64
# end

# function MiniBee_cst(aircraft::avion,Cl= 1., Cd=0.0270, Ct=1., Sw=16.2, C_D_alpha2=0., Sf=15., C_L_alpha=10., C_D0=0.0095, e_os=0.75, C_C_beta=-2.0)
#     AR = 7.40 #aircraft.width/Sw
#     C_D_alpha2 = C_L_alpha^2/(pi*e_os*AR)
#     set_cst = MiniBee_cst(Cl, Cd, Ct, Sw, AR, C_D_alpha2, Sf, C_L_alpha, C_D0, e_os, C_C_beta)
#     return set_cst
# end


