using DifferentialEquations
using Plots
include("model_ode.jl")
include("ode_rk4.jl")
include("score_function.jl")

############
# Parameters 
############

# Constants of the aircraft
physical_data = physic_cst()
aircraft = avion()
aircraft_cst = MiniBee_cst(aircraft)

# for resolution
X0 = SA[0,0,0,1,1,1,0,0,0,0,3000]

Tmax = 1.
T0 = 0.
dt = 0.01

# for visualisation
plot_3D = true
plot_coord = true

plot_3D_rk4 = true
plot_coord_rk4 = true

# for assessment
x_end_fake = SA[0.85, 0, 0.25]

# useful Functions 
function Ut(t,m)
    if m > aircraft.dry_mass 
        return SA[250,100,100,100]
    else 
        return SA[0,100,100,100]
    end
end

function plot_traj_3d(trajectory)
    """
    Do : Plot the three coordinates 
    Input : 
        - t : time (N)
        - trajectory [4,N], trajectory on x,y,z and the mass on N steps.
    """
    plt = plot3d(
        1,
        title = "3D Trajectory",
        marker = 2,
    )
    @gif for i=1:size(trajectory)[2]
        push!(plt,trajectory[1,i], trajectory[2,i], trajectory[3,i])
        end every 1
    @show(plt)
end

function plot_traj_3dcoords(t,position)
    """
    Do : Plot the three coordinates 
    Input : 
        - t : time (N)
        - position [4,N], trajectory on x,y,z and the mass on N steps.
    """
    plt_coord = plot(t,position[1,:], title="Trajectory on the different axes")
    plot!(t,position[2,:])
    plot!(t,position[3,:])
    @show plt_coord
end 

function plot_traj_3dcoords_rk4(t,position)
    """
    Do : Plot the three coordinates 
    Input : 
        - t : time (N)
        - position [4,N], trajectory on x,y,z and the mass on N steps.
    """
    plt_coord = plot(t,position[:,1], title="Trajectory on the different axes")
    plot!(t,position[:,2])
    plot!(t,position[:,3])
    @show plt_coord
end 




#########################################################
####            Version with julia solver            ####
#########################################################

f_ode = ODEFunction(f)
t_span = (T0,Tmax) 

println("Time Julia Solver : ")
@time begin
    problem_ode = ODEProblem(f_ode,X0,t_span,Ut)
    sol = solve(problem_ode)
end


println("Score function : ", J(sol,x_end_fake,X0,Tmax,"Julia_sover"))
trajectory = sol[1:3,:]
println(size(sol))


if plot_3D
    plot_traj_3d(trajectory)
end

if plot_coord
    plot_traj_3dcoords(sol.t,trajectory) 
end 
########################################################
###                Version with RK4                 ####
########################################################

U0 = Ut

println("Temps RK4 : ")
@time x_stockage = RK4(T0,Tmax,dt,X0,U0,f)

println(typeof(x_stockage),size(x_stockage))
println("Solution : ", x_stockage)

println("Score function : ", J(x_stockage,x_end_fake,X,dt, "RK4"))

println(size(x_stockage))
if plot_3D_rk4
    plot_traj_3d(x_stockage)
end

if plot_coord_rk4
    t = collect(0:1:1*size(x_stockage)[1]-1)*dt
    plot_traj_3dcoords_rk4(t,x_stockage)
end