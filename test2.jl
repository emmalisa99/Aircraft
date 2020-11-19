using DifferentialEquations
using Plots
include("model_ode.jl")
include("ode_rk4.jl")
include("score_function.jl")

# Constants of the aircraft
physical_data = physic_cst()
aircraft = avion()
aircraft_cst = MiniBee_cst(aircraft)


#########################################################
####                Version with RK4                 ####
#########################################################

function U_t(t)
    return SA[1000000,0,10,0]
end

# initialisation
X_0 = SA[0,0,0,1,1,1,0,0,0,0,300]
U0 = U_t
# parameters of rk4
t = 0.
T = 1.
dt = 0.01

#resolution
println("Temps RK4 : ")
@time x_stockage = RK4(t,T,dt,X,U0,f)

println("Solution : ", x_stockage)
