using StaticArrays

struct avion
    dry_mass::Float64   #Mass without fuel 
    kt::Float64         #Constant linked to the type of aircraft 
    max_thrust::Float64 #U1 maximum
    max_amplitude_anguar_control::Float64   #U2,U3,U4 maximum
    length::Float64
    width::Float64
    thickness::Float64

    avion(dry_mass = 500., kt=1., max_thrust=1e3, max_amplitude_angular_control=10., length=4., width=1.5, thickness=0.7) = new(dry_mass , kt, max_thrust, max_amplitude_angular_control, length, width, thickness)
end

struct physic_cst 
    g::SVector{3, Float64}      #Gravity vector
    r::Float64  #Air perfect gas specific constant 
    T::Float64  #Temperature
    physic_cst(g = SA[0,0,9.81], r = 287., T = 288.15) = new(g,r,T)
end 

#To improve : add C_D_alpha2 and AR (problems with connections between structures)
struct MiniBee_cst 
    Cl::Float64     #Lift coefficient
    Cd::Float64     #Drag coefficient 
    Ct::Float64     

    Sw::Float64     #Wing area
    Sf::Float64     #Frontol area

    C_L_alpha::Float64
    C_D0::Float64
    e_os::Float64
    C_C_beta::Float64

    MiniBee_cst(Cl= 0.15, Cd=0.05, Ct=1., Sw=0.63, Sf=15., C_L_alpha=10, C_D0=0.0095, e_os=0.7, C_C_beta=-2.0) = new(Cl, Cd, Ct, Sw, Sf, C_L_alpha, C_D0, e_os, C_C_beta)
end 

