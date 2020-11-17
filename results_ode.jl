using DifferentialEquations
using Plots
include("model_ode.jl")
include("ode_rk4.jl")
include("score_function.jl")

physical_data = physic_cst()
aircraft = avion()
aircraft_cst = MiniBee_cst(aircraft)


#########################################################
####            Version with julia solver            ####
#########################################################
f_ode = ODEFunction(f) # y a t il d'autres truc à mettre ? 
u_0 = SA[0,0,0,1,0,0,0,0,0,15,300]
Tmax = 1
t_span = (0,Tmax)
p = SA[50,0,0,0]

println("Time Julia Solver : ")
@time begin
    problem_ode = ODEProblem(f_ode,u_0,t_span,p)
    sol = solve(problem_ode)
end

println(sol)
x_end_fake = SA[0.85, 0, 0.25]
println("Score function : ", J(sol,x_end_fake,u_0,Tmax,"Julia_sover"))
trajectory = sol[1:3,:]

plt = plot3d(
    1,
    title = "Trajectory",
    marker = 2,
)

@gif for i=1:size(trajectory)[2]
     push!(plt,trajectory[1,i], trajectory[2,i], trajectory[3,i])
 end every 20

@show(plt)

plt_coord = plot(trajectory[1,:], title="trajectory on the different axes")
plot!(trajectory[2,:])
plot!(trajectory[3,:])




#########################################################
####                Version with RK4                 ####
#########################################################

# initialisation
X = SA[0,0,0,1,0,0,0,15,0,0,300]
U0 = SA[100,0,20,0]
# parameters of rk4
t = 0.
T = 0.2
dt = 0.01
step = 0.1
#resolution

println("Temps RK4 : ")
@time x_stockage = RK4(t,T,dt,X,U0,f)

println("Solution : ", x_stockage)

x_end_fake = SA[0.85, 0, 0.25]
println("Score function : ", J(X_stockage,x_end_fake,X,dt, "RK4"))

plt = plot3d(
    1,
    title = "Trajectory",
    marker = 2,
)

@gif for i=1:size(trajectory)[1]
     push!(plt,x_stockage[i,1]) #, x_stockage[i,2], x_stockage[i,3])
 end every 20

@show(plt)
plt_coord = plot(x_stockage[:,1], title="trajectory on the different axes - rk4")
plot!(x_stockage[:,2])
plot!(x_stockage[:,3])