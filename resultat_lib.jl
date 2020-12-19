##################################
#####      Solver Julia      #####
##################################
function solve_problem(Tmax, X0, p, f)
    t_span = (0.,Tmax)
    problem_ode = ODEProblem(f,[X0...],t_span,p,dtmax=0.1)
    solve(problem_ode)
end


##################################
#####     Benchhmarking      #####
##################################
function make_benchmark(fonction,argument,titre)
    @info "Benchmarking"
    @info titre
    println(@benchmark fonction(argument...))
end


##################################
#####        Affichage       #####
##################################

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
    return plt
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
    return plt_coord
end


function plot_3D_option_forces(sol, p, option_forces=False, nb_forces=10)
    trajectory = hcat(sol.u...)
    plt = plot3d(
        1,
        title = "3D Trajectory",
        marker = 2,
    )
    plt = plot3d(trajectory[1,:], trajectory[2,:], trajectory[3,:],label="Trajectoire")

    if option_forces
        d_inter = Int(floor(size(sol.t)[1]/nb_forces))
        for i=1:nb_forces
            t = sol.t[i*d_inter]
            force = forces(sol.u[i*d_inter],p,t)
            pos = force.Position
            masse = force.Mass
            lift = force.Lift/masse
            drag = force.Drag/masse
            poussee = force.Thrust/masse
            plot3d!(plt, [pos[1],pos[1]+lift[1]],[pos[2],pos[2]+lift[2]],[pos[3],pos[3]+lift[3]],color="blue",label=nothing)
            plot3d!(plt, [pos[1],pos[1]+drag[1]],[pos[2],pos[2]+drag[2]],[pos[3],pos[3]+drag[3]],color="red",label=nothing)
            plot3d!(plt, [pos[1],pos[1]+poussee[1]],[pos[2],pos[2]+poussee[2]],[pos[3],pos[3]+poussee[3]],color="green",label=nothing)
        end
    end
    @show plt
    savefig("3d.png")
end


function plot_trajectory_projection(sol,p,axe1,axe2,option_forces=False,nb_forces=10)
    Axes = Dict("x"=>1, "y"=>2, "z"=>3)
    #Projection Axe1 , Axe2
    name_projection = axe1*axe2
    axe1 = Axes[axe1]
    axe2 = Axes[axe2]
    trajectory = hcat(sol.u...)
    plt_proj = plot(trajectory[axe1,:],trajectory[axe2,:],label="Proj sur "*name_projection)

    if option_forces
        d_inter = Int(floor(size(sol.t)[1]/nb_forces))
        for i in 1:n_inter
            t = sol.t[i*d_inter]
            force = forces(sol.u[i*d_inter],p,t)
            pos = force.Position
            mass = force.mass
            lift = force.Lift/mass
            drag = force.Drag/mass
            poussee = force.Thrust/mass
            plot!(plt_proj, [pos[1],pos[1]+lift[1]],[pos[2],pos[2]+lift[2]],color="blue",label=nothing)
            plot!(plt_proj, [pos[1],pos[1]+drag[1]],[pos[2],pos[2]+drag[2]],color="red",label=nothing)
            plot!(plt_proj, [pos[1],pos[1]+poussee[1]],[pos[2],pos[2]+poussee[2]],color="green",label=nothing)
        end
    end
    @show plt_proj
    savefig("proj"*name_projection*".png")
end




function plot_3D_option_forces_rk4(trajectory, life_span, p, option_forces=False, nb_forces=10)
    plt = plot3d(
        1,
        title = "3D Trajectory",
        marker = 2,
    )
    print(trajectory[1,1], trajectory[1])
    plt = plot3d(trajectory[1,:], trajectory[2,:], trajectory[3,:],label="Trajectoire")

    if option_forces
        d_inter = Int(floor(size(life_span)[1]/nb_forces))
        for i=1:nb_forces
            t = life_span[i*d_inter]
            force = forces(trajectory[i*d_inter,:],p,t)
            pos = force.Position
            masse = force.Mass
            lift = force.Lift/masse
            drag = force.Drag/masse
            poussee = force.Thrust/masse
            plot3d!(plt, [pos[1],pos[1]+lift[1]],[pos[2],pos[2]+lift[2]],[pos[3],pos[3]+lift[3]],color="blue",label=nothing)
            plot3d!(plt, [pos[1],pos[1]+drag[1]],[pos[2],pos[2]+drag[2]],[pos[3],pos[3]+drag[3]],color="red",label=nothing)
            plot3d!(plt, [pos[1],pos[1]+poussee[1]],[pos[2],pos[2]+poussee[2]],[pos[3],pos[3]+poussee[3]],color="green",label=nothing)
        end
    end
    @show plt
    savefig("3d.png")
end


function plot_trajectory_projection_rk4(trajectory,life_span,p,axe1,axe2,option_forces=False,nb_forces=10)
    Axes = Dict("x"=>1, "y"=>2, "z"=>3)
    #Projection Axe1 , Axe2
    name_projection = axe1*axe2
    axe1 = Axes[axe1]
    axe2 = Axes[axe2]
    plt_proj = plot(trajectory[axe1,:],trajectory[axe2,:],label="Proj sur "*name_projection)

    if option_forces
        d_inter = Int(floor(size(life_span)[1]/nb_forces))
        for i in 1:n_inter
            t = life_span[i*d_inter]
            force = forces(trajectory[i*d_inter,:],p,t)
            pos = force.Position
            mass = force.mass
            lift = force.Lift/mass
            drag = force.Drag/mass
            poussee = force.Thrust/mass
            plot!(plt_proj, [pos[1],pos[1]+lift[1]],[pos[2],pos[2]+lift[2]],color="blue",label=nothing)
            plot!(plt_proj, [pos[1],pos[1]+drag[1]],[pos[2],pos[2]+drag[2]],color="red",label=nothing)
            plot!(plt_proj, [pos[1],pos[1]+poussee[1]],[pos[2],pos[2]+poussee[2]],color="green",label=nothing)
        end
    end
    @show plt_proj
    savefig("proj"*name_projection*".png")
end 