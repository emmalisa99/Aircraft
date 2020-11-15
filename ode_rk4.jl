include("model_ode.jl")

function iter_RK4(X,U,step,f)
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

function RK4(t,T,dt,X,U0,step,f)
    x_stockage = zeros(Float64, 20, 3)
    while t < T  
        t = t + dt 
        X = iter_RK4(X,U0,step,f)
        x_stockage[t,:] = X[1:3]
    end
    return x_stockage
end

function test()
    # initialisation
    X = SA[0,0,0,1,0,0,0,15,0,0,300]
    U0 = SA[100,0,0,0]
    step = 0.01
    # parameters of rk4
    t = 0 
    T = 20
    dt = 1
    #resolution
    x_stockage = RK4(t,T,dt,X,U0,step,f)
    println("Solution : ", x_stockage)
end
