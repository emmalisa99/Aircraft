include("model_ode.jl")

function iter_RK4(X,U,step::Float64,f)
    k1 = f(X,U)
    k2 = f(X + k1*step/2, U)
    k3 = f(X + k2*step/2, U)
    k4 = f(X + k3*step, U)

    next_states = X  + (k1 + 2*k2 + 2*k3 + k4)*step/6

    return next_states
end

function test_RK4()
    X = SA[0,0,0,1,0,0,0,15,0,0,300]
    U0 = SA[100,0,0,0]
    step = 0.01
    solution = iter_RK4(X,U0,step, f)
    println("X = ", solution)
end

function U_t(t)
    return SA[100,0,0,0]
end

function RK4(t::Float64,T::Float64,dt::Float64,X,U0,f)
    nb_iter = Int(T/dt)
    x_stockage = []
    iter = 1
    while t < T  && X[11] > aircraft.dry_mass
        t = t + dt 
        X = iter_RK4(X,U0,dt,f)
        push!(x_stockage, SA[X[1],X[2], X[3], X[11]])
        iter += 1
    end
    return x_stockage # pos_x, pos_y, pos_z, m
end

function test()
    # initialisation
    X = SA[0,0,0,1,0,0,0,15,0,0,300]
    U = U_t#SA[100,0,0,0]
    # parameters of rk4µ
    t = 0.
    T = 0.2
    dt = 0.01
    #resolution
    x_stockage = RK4(t,T,dt,X,U,f)
    println("Solution : ", x_stockage)
end