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
####            Version with julia solver            ####
#########################################################
f_ode = ODEFunction(f) # y a t il d'autres truc à mettre ? 
X_0 = SA[0,0,0,1,1,0,0,0,0,0,300]
function Ut(t)
    if t < 0.5
        return SA[0,0,0,0]
    elseif t < 1.3
        return SA[1000000,0,0,0]
    else
        return SA[0,0,0,0]
    end
end
Tmax = 1
t_span = (0,Tmax) 

println("Time Julia Solver : ")
@time begin
    problem_ode = ODEProblem(f_ode,X_0,t_span,Ut)
    sol = solve(problem_ode)
end

x_end_fake = SA[0.85, 0, 0.25]
println("Score function : ", J(sol,x_end_fake,X_0,Tmax,"Julia_sover"))
trajectory = sol[1:3,:]
println(size(sol))

plot_3D = false
plot_coord = false
if plot_3D
    plt = plot3d(
        1,
        title = "Trajectory",
        marker = 2,
    )

    @gif for i=1:size(trajectory)[2]
        push!(plt,trajectory[1,i], trajectory[2,i], trajectory[3,i])
        end every 1

    @show(plt)
end

if plot_coord
    plt_coord = plot(sol.t,trajectory[1,:], title="trajectory on the different axes")
    plot!(sol.t,trajectory[2,:])
    plot!(sol.t,trajectory[3,:])
    @show plt
end 

#########################################################
####                Version with RK4                 ####
#########################################################

function U_t(t)
    return SA[1000000,0,10,0]
end

# initialisation
X = SA[0,0,0,1,1,1,1,1,1,1,300]
U0 = U_t
# parameters of rk4
t = 0.
T = 1.
dt = 0.01

#resolution
println("Temps RK4 : ")
@time x_stockage = RK4(t,T,dt,X,U0,f)

println("Solution : ", x_stockage)

x_end_fake = SA[0.85, 0, 0.25]
println("Score function : ", J(x_stockage,x_end_fake,X,dt, "RK4"))


plot_3D_rk4 = true
plot_coord_rk4 = true

if plot_3D_rk4
    plt = plot3d(
        1,
        title = "Trajectory",
        marker = 2,
    )
    @gif for i=1:size(x_stockage)[1]
        push!(plt,x_stockage[i,1], x_stockage[i,2], x_stockage[i,3])
    end every 1
    @show(plt)
    savefig("trajectory_3d_rk4")
end

if plot_coord_rk4
    plt_coord = plot(x_stockage[:,1], title="trajectory on the different axes - rk4")
    plot!(x_stockage[:,2])
    plot!(x_stockage[:,3])
    savefig("trajectory_rk4")
end