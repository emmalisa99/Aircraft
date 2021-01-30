include("model_ode.jl")

function iter_RK4(dX, X,p,t,step::Float64,f)
    k1 = f(dX, X, p, t)
    k2 = f(dX, X + k1*step/2, p, t)
    k3 = f(dX, X + k2*step/2, p, t)
    k4 = f(dX, X + k3*step, p, t)

    next_states = X  + (k1 + 2*k2 + 2*k3 + k4)*step/6

    return next_states
end

function RK4(t::Float64,T::Float64,dt::Float64,dX,X,p,f)
    nb_iter = Int(floor(T/dt))+1
    x_stockage = zeros(Float64,4,nb_iter)
    x_stockage[:,1] = SA[X[1],X[2],X[3],X[11]]
    iter = 1
    while t < T  
        t = t + dt 
        X = iter_RK4(dX, X, p, t, dt, f)
        x_stockage[:,iter] =  @SVector [X[1],X[2],X[3],X[11]]
        iter += 1
    end
    return x_stockage # pos_x, pos_y, pos_z, m
end


##########################
##        test          ##
##########################

function test_RK4()
    X = SA[0,0,0,1,0,0,0,15,0,0,300]
    dX = similar(X0)
    U0 = SA[100,0,0,0]
    p = (physical_data, aircraft, U0)
    step = 0.01
    t = 0
    solution = iter_RK4(dX, X, p, t, step, f)
    println("X = ", solution)
end

function U_t(t,m)
    return SA[100,0,0,0]
end


function test()
    # initialisation
    X = SA[0,0,0,1,0,0,0,15,0,0,300]
    U = U_t
    p = (physical_data, aircraft, Ut)
    # parameters of rk4µ
    t = 0.
    T = 0.2
    dt = 0.01
    #resolution
    x_stockage = RK4(t,T,dt,dX,X,p,f)
    println("Solution : ", x_stockage)
end