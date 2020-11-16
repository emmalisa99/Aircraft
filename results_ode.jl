using DifferentialEquations
using Plots
include("model_ode.jl")
include("ode_rk4.jl")

physical_data = physic_cst()
aircraft = avion()
aircraft_cst = MiniBee_cst(aircraft)


#########################################################
####            Version with julia solver            ####
#########################################################
f_ode = ODEFunction(f) # y a t il d'autres truc à mettre ? 
X_0 = SA[0,0,0,10,0,0,15,0,0,0,300]
function Ut(t)
    if t < 1
        return SA[500,0,0,0]
    else 
        return SA[500,0,0,0]
    end
end

t_span = (0,1) 

@time begin
    problem_ode = ODEProblem(f_ode,X_0,t_span,Ut,dtmax=t_span[2]/100)
    sol = solve(problem_ode)
end

println(sol)

trajectory = sol[1:3,:]

plt = plot3d(
    1,
    title = "Trajectory",
    marker = 2,
)

@gif for i=1:size(trajectory)[2]
      push!(plt,trajectory[1,i], trajectory[2,i], trajectory[3,i])
    end every 1

# @show(plt)

plt_coord = plot(trajectory[1,:], title="trajectory on the different axes")
plot!(trajectory[2,:])
plot!(trajectory[3,:])

#@show plt


#########################################################
####                Version with RK4                 ####
#########################################################
function U_t(t)
    return SA[100,0,0,0]
end

# initialisation
X = SA[0,0,0,1,0,0,0,15,0,0,300]
U0 = U_t#SA[0,0,1.1,0]
# parameters of rk4
t = 0.
T = 0.1
dt = 0.01
# resolution
@time x_stockage = RK4(t,T,dt,X,U0,f)

println("Solution : ", x_stockage)


plt = plot3d(
    1,
    title = "Trajectory",
    marker = 2,
)

@gif for i=1:size(trajectory)[1]
     push!(plt,x_stockage[i,1], x_stockage[i,2], x_stockage[i,3])
 end every 20

@show(plt)

plt_coord = plot(x_stockage[:,1], title="trajectory on the different axes - rk4")
plot!(x_stockage[:,2])
plot!(x_stockage[:,3])