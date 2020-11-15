Project MiniBee

# Objective

Find the optimal flight plan of an aircraft to go from a point A to B. We try to find the path that take the less time and/or that consumes the less carburant. The optimal path has to respect some contraints. Some are physical, others impose to avoid cities. 

# Requirements

DifferentialEquations.jl
Plots.jl
LinearAlgebra.jl
StaticArrays.jl

# Architecture

debut.jl : define aircraft structure and physical constante

model_ode : define the physical model (under the form of an ode)
ode_rk4 : propose the RK4 method to resolve an ode
results_ode : propose a resolution of the physical model by the rk4 or the julia solver 