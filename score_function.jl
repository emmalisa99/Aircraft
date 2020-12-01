using LinearAlgebra
include("ode_rk4.jl")
include("values.jl")

function J(X_storage,x_final,X_0,time,method)
    """
    Quand method = "RK4", time représente dt
    Quand method = "Julia_solver", time représente le temps de trajet
    """
    Tmax = size(X_storage)[1]
    if method == "RK4"
        trajectory_time = time * Tmax
    else
        trajectory_time = time
    end
    fuel_consumtion = abs(X_storage[Tmax,4]- X_0[11])
    is_finalState = norm(X_storage[Tmax,1:3] - x_final)
    J = is_finalState + fuel_consumtion + trajectory_time
    return J
end

#### faudrait peut etre modifier le rk4 pour que s'arrete selon lieu ? ---> question a se poser sur le x_final

# initialisation
U0 = @SArray [100,0,0,0]
step = 0.01
# parameters of rk4
t = 0.0
T = 0.2
dt = 0.01
#resolution
# @time X_stockage = RK4(t,T,dt,X0,U0,f)
# X_final = @SVector [0.791827;   0.0177039;    0.219864]


# println("Test : ")
# println("Attendu : (fuel)20+(dist)0+(temps)0.2")
# println("Score = ",J(X_stockage,X_final,X0,dt, "RK4"))
