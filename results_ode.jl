using DifferentialEquations
using Plots
using BenchmarkTools
include("model_ode.jl")
include("ode_rk4.jl")
include("angles.jl")
include("values.jl")

############
# Parameters
############



# for visualisation
plot_3D = true
plot_coord = true

plot_3D_rk4 = false
plot_coord_rk4 = false

# for assessment
x_end_fake = @SVector [0.85, 0, 0.25]

# useful Functions
function Ut(t,X)
    if X[3] < 25 || X[6] <0
        U = @SArray [70000,0,0,0]
    else
        U = @SArray [0,0,0,0]
    end


    if X[11] <= aircraft.dry_mass
        U[1] = 0
    end
    return U
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


"""ODE Problem : inclure un dx comme un array pré-allouer dans lequel on met nos résultats
(vecteur de résultats) / le solveur gère tout seul l'allocation des temps intermédiaires """

# println("Score function : ", J(sol,x_end_fake,X0,Tmax,"Julia_sover"))
# trajectory = sol[1:3,:]
# println(size(sol))

function solve_problem(Tmax, p)
    t_span = (0.,Tmax)
    # println("Time Julia Solver : ")
    problem_ode = ODEProblem(f,[X0...],t_span,p)
    solve(problem_ode)
end

const p = (physical_data, aircraft, Ut)
dX = similar(X0)

@info "Benchmarking"
@info "Derivative"
println(@benchmark f($dX, $X0, $p, 1.0))
@info "ODE solve"
println(@benchmark solve_problem($Tmax, $p))

sol = solve_problem(Tmax, p)
trajectory = hcat((sol.(0:0.01:2))...)

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

# println("Temps RK4 : ")
# @time x_stockage = RK4(T0,Tmax,dt,X0,U0,f)
#
# println(typeof(x_stockage),size(x_stockage))
# println("Solution : ", x_stockage)
#
# println("Score function : ", J(x_stockage,x_end_fake,X,dt, "RK4"))

# println(size(x_stockage))
# if plot_3D_rk4
#     plot_traj_3d(x_stockage)
# end
#
# if plot_coord_rk4
#     t = collect(0:1:1*size(x_stockage)[1]-1)*dt
#     plot_traj_3dcoords_rk4(t,x_stockage)
# end
