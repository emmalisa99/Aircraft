using StaticArrays
# using Ipopt

struct physic_cst 
    g::SVector{3, Float64}      #Gravity vector
    r::Float64  #Air perfect gas specific constant 
    T::Float64  #Temperature
    physic_cst(g = SA[0,0,9.81], r = 287., T = 288.15) = new(g,r,T)
end


struct avion
    dry_mass::Float64   #Mass without fuel 
    kt::Float64         #Constant linked to the type of aircraft 
    max_thrust::Float64 #U1 maximum
    max_amplitude_anguar_control::Float64   #U2,U3,U4 maximum
    length::Float64
    width::Float64
    thickness::Float64

    avion(dry_mass = 200., kt=1., max_thrust=1e3, max_amplitude_angular_control=10., length=4., width=1.5, thickness=0.7) = new(dry_mass , kt, max_thrust, max_amplitude_angular_control, length, width, thickness)
end

 

#To improve : add C_D_alpha2 and AR (problems with connections between structures)
mutable struct MiniBee_cst 
    Cl::Float64     #Lift coefficient
    Cd::Float64     #Drag coefficient 
    Ct::Float64     

    Sw::Float64     #Wing area
    AR              #
    C_D_alpha2
    Sf::Float64     #Frontol area

    C_L_alpha::Float64
    C_D0::Float64
    e_os::Float64
    C_C_beta::Float64

    function MiniBee_cst(aircraft::avion,Cl= 0.15, Cd=0.05, Ct=1., Sw=0.63, AR=nothing, C_D_alpha2=nothing, Sf=15., C_L_alpha=10, C_D0=0.0095, e_os=0.7, C_C_beta=-2.0) 
        set_cst = new(Cl, Cd, Ct, Sw, AR, C_D_alpha2, Sf, C_L_alpha, C_D0, e_os, C_C_beta)
        set_cst.AR = aircraft.width/Sw
        set_cst.C_D_alpha2 = C_L_alpha^2/(pi*e_os*set_cst.AR)
        return set_cst
    end
end 

function pressure(height)
    p = 1013.25 * (1 - 0.0065*height * (1.0/288.15))^(5.255)
    return p
end

function rho(height, pressure, r, T)
    return 100 * pressure(height) * (1.0 / (r * T))
end 


av = avion()
cst = MiniBee_cst(av)
println(cst)

"""
Partie optimisation
"""

# function eval_f(X,U,tf)    #Fonction coût
#     return 
# end

# function eval_g(X,U,tf)     #Fonction contraintes
#     return
# end

# function eval_grad_f(X,U)   #A voir : utilisation de modules de différenciation ?
#     return
# end

# function eval_jac_g(X,U,tf)     #A voir : rows, values, cols ? (qu'est ce que c'est ?)
#     return
# end

# function eval_h(X,U,tf)     #peut être pas nécessaire
#     return 
# end 

# prob = createProblem(n, x_L, x_U, m, g_L, g_U, 8, 10, eval_f, eval_g, eval_grad_f, eval_jac_g, eval_h)
# prob.x =    #Set starting solution
# status = solveProblem(prob)

# println(Ipopt.ApplicationReturnStatus[status])
# println(prob.x)
# println(prob.obj_val)
