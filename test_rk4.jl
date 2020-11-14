include("ode.jl")
function RK4(X,U,step,f)
    k1 = f(X,U)
    k2 = f(X + k1*step/2, U)
    k3 = f(X + k2*step/2, U)
    k4 = f(X + k3*step, U)

    next_states = X  + (k1 + 2*k2 + 2*k3 + k4)*step/6

    return next_states
end

X = SA[0,0,0,0,0,0,0,0,0,0,300]
U0 = SA[100,0,0,0]
step = 0.1
solution = RK4(X,U0,step, f)
