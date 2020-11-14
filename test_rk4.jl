include("ode.jl")
function RK4(X,U,step,f)
    k1 = f(X,U)
    k2 = f(X + k1*step/2, U)
    k3 = f(X + k2*step/2, U)
    k4 = f(X + k3*step, U)

    next_states = X  + (k1 + 2*k2 + 2*k3 + k4)*step/6

    return next_states
end

X = SA[0,0,0,1,0,0,0,15,0,0,300]
U0 = SA[100,0,0,0]
step = 0.01
solution = RK4(X,U0,step, f)
println("X = ", solution)

t = 0 
T = 20
dt = 1
x_stockage = zeros(Float64, 20, 3)
while t < T  
    t = t + dt 
    X = RK4(X,U0,step,f)
    x_stockage[t,:] = X[1:3]
end

println("solution : ", x_stockage)