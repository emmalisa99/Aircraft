using DifferentialEquations
using Plots
using BenchmarkTools
using StaticArrays
include("quaternions_lib.jl")
include("structure_and_data.jl")
include("angle_lib.jl")
include("coefficients.jl")
include("resultat_lib.jl")
include("model_ode.jl")
include("ode_rk4.jl")
include("initialisation.jl")



############
# Parameters
############

# for visualisation and test
test_benchmark = false

julia_solver = true
plot_3D = true
option_forces = false
plot_coord = true

rk4_solver = false
plot_3D_rk4 = true
option_forces_rk4 = false
plot_coord_rk4 = true

# for assessment
x_end_fake = @SVector [0.85, 0, 0.25]


#########################################################
####            Version with julia solver            ####
#########################################################

"""ODE Problem : inclure un dx comme un array pré-allouer dans lequel on met nos résultats
(vecteur de résultats) / le solveur gère tout seul l'allocation des temps intermédiaires """

if julia_solver 
    #angle_function = find_angle_spline(1,Angle,angle_echelon,1)#find_angle_polynome(Angle,angle_echelon,1,5,2/3,0.01)
    angle_function = find_angle_spline(1,Angle,angle_echelon,1)#find_angle_polynome(Angle,angle_echelon,1,3,1/5,0.8)
    p = (aircraft_physical_data..., control, angle_function)
    dX = similar(X0)

    if test_benchmark
        make_benchmark(f,(dX, X0, p, 1.0),"Iteration sur f")
        make_benchmark(solve_problem,(Tmax, X0, p, f),"ODE solve")
    end

    sol = solve_problem(Tmax, X0, p, f_turn)
    trajectory = hcat(sol.u...)
    for t=1:size(sol.t)[1]
        for i=1:3
            if abs(sol.u[t][i]) < 1e-12
                sol.u[t][i] = 0
            end
        end
    end

    if plot_3D
        plot_3D_option_forces(sol, p, option_forces)
    end

    if plot_coord
        #plt = plot_traj_3dcoords(sol.t,trajectory)

        #Projection xz
        plot_trajectory_projection(sol,p,"x","y",option_forces)

        #Projection xz
        plot_trajectory_projection(sol,p,"x","z",option_forces)
    end

    # println("Score function : ", J(sol,x_end_fake,X0,Tmax,"Julia_sover"))
    
end


########################################################
###                Version with RK4                 ####
########################################################

if rk4_solver
    angle_function = find_angle_polynome(Angle,angle_echelon,1,1.3,0.12,0.005)
    p = (aircraft_physical_data..., control, angle_function)
    dX = similar(X0)

    println("Temps RK4 : ")
    @time x_stockage = RK4(T0,Tmax,dt,dX,X0,p,f)

    for t=1:size(x_stockage)[2]
        for i=1:3
            if abs(x_stockage[i,t]) < 1e-10
                x_stockage[i,t] = 0
            end
        end
    end

    #println(typeof(x_stockage),size(x_stockage))
    #println("Solution : ", x_stockage)

    #println("Score function : ", J(x_stockage,x_end_fake,X,dt, "RK4"))

    # println(size(x_stockage))
    # if plot_3D_rk4
    #     plot_traj_3d(x_stockage')
    # end

    # if plot_coord_rk4
    #     t = collect(0:1:1*size(x_stockage)[1]-1)*dt
    #     plot_traj_3dcoords_rk4(t,x_stockage)
    # end

    if plot_3D_rk4
        t = collect(0:dt:Tmax)
        plot_3D_option_forces_rk4(x_stockage, t, p, option_forces)
    end

    if plot_coord_rk4
        #plt = plot_traj_3dcoords(sol.t,trajectory)
        t = collect(0:dt:Tmax)
        #Projection xz
        plot_trajectory_projection_rk4(x_stockage,t,p,"x","y",option_forces)

        #Projection xz
        plot_trajectory_projection_rk4(x_stockage,t,p,"x","z",option_forces)
    end
end

