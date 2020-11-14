using DifferentialEquations
include("debut.jl")


physical_data = physic_cst()
aircraft = avion()
aircraft_cst = MiniBee_cst(aircraft)

# X = [x1,x2,x3,  v1,v2,v3,  q1,q2,q3,q4, m]
# U = [U1,  U2,U3,U4]


function f(X,U,t)  ## pb avec autres paramètres
    
    g = physical_data.g                               # gravity
    kt = aircraft.kt                                     # constant relative to the aircraft
    
    i =  SA[ 2*(X[7]^2+ X[8]^2)-1 , 
             2*(X[8]*X[9]+X[7]*X[10]) ,
             2*(X[8]*X[10]-X[7]*X[9]) ]               # effect of the rotation on speed
    
    M = [0 -U[2] -U[3] -U[4] ;
        U[2] 0 U[4] -U[3] ; 
        U[3] -U[4] 0 U[2] ; 
        U[4] U[3] -U[2] 0]                            # matrix for quaternion 
    
    norm2_speed = (X[4]^2 + X[5]^2 + X[6]^2) 
    alpha = asin(-X[6]/norm2_speed) 
    beta = asin(-X[5]/norm2_speed)
    ca = cos(alpha)
    cb = cos(beta)
    sa = sin(alpha)
    sb = cos(beta)
    P_B2W = [ ca*cb -ca*sb sa;
            sb cb 0;
            -sa*cb sa*sb ca]                           # "passage" matrix to wind coordinate to aircraft coordinate 
    C_D = aircraft_cst.C_D0  + aircraft_cst.C_D_alpha2*alpha^2
    Cst_DCL = SA[-C_D -aircraft_cst.C_C_beta aircraft_cst.C_L_alpha ] 
    eta_a = debut.rho[X[3]] * Sw * 0.5 
    Fa = eta_a * norm2_speed * P_B2W * Cst_DCL         # aerodynamical forces

    f = Vector(undef, 11)
    f[1:4] = X[4:7]                                    # dot x = v
    f[4:7] = Fa/m + g + U[1]/m * (kt*v + i)            # dot v = sum of forces (principle of dynamics)
    f[7:11] = 1/2 * M * q
    f[11] = -kt * U[1]                                 # dot m = -kt * trhust : variation of fuel

end

f_ode = ODEFunction(f) # y a t il d'autres truc à mettre ? 
u_0 = SA[0,0,0,0,0,0,0,0,0,0,0]
t_span = (0,0.5)
p = SA[1,0,0,0]


problem_ode = ODEProblem(f_ode,u_0,t_span,p)
sol = solve(problem_ode)

