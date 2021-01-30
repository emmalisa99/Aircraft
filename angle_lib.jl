using Polynomials
using DataInterpolations
using Dierckx
using Plots 
include("equations_equilibre.jl")


####################################################################
## Find an equation that change the intial angle to an echelon angle
#################################################################### 

function find_angle_polynome(theta1,theta2,t,tf,x_pt,a_pt)
    """
    Obj : creer une fonction polynome pour aller d'un angle theta 1 à un angle theta2 sur une durée de tf-t.
    Input : 
        - theta1 : angle de départ
        - theta2 : angle d'arrivée
        - t : temps signalant le début du changement d'anglemeny
        - tf : temps signalant l'angle où on atteint l'angle theta2
        - x_pt : abscisse en pourcentage pour placer le 3e point 
        - a_pt : ordonnée en pourcentage pour placer le 3e point
    """
    d_angle = abs(theta1 - theta2)
    t = [t, t+tf, t+(tf-t)*x_pt]
    angle = [theta1, theta2, theta2+d_angle*a_pt]
    return fit(t,angle,2)
end

function find_angle_spline(d_temps,angle,angle_echelon,t_init)
    """
    Obj : creer une fonction spline pour aller d'un angle theta 1 à un angle theta2 sur une durée d_temps.
    Input : 
        - angle : angle de départ
        - angle_echelon : angle d'arrivée
        - t _init: temps signalant le début du changement d'anglement
        - d_temps : durée pour atteindre l'angle échelon
    """
    temps = [0,2.5,5,6.1,10,12,15.5,16]
    ord = [0.,5.,11.,12.,10.,9.5,10.,10.]
    len = size(temps)[1]

    d_angle = (angle_echelon - angle) 
    temps = t_init * ones(Float64,len) + temps*d_temps/(temps[len]-temps[1]) # + 1
    ord = angle*ones(Float64,len) + ord * d_angle/(ord[len]-ord[1]+1) 
    
    return Spline1D(temps,ord) 
end


function test_function_angles(fonction, angle, angle_echelon)
    intervalle = 0,11
    dt = 0.01
    temps = collect(intervalle[1]:dt:intervalle[2])
    len = size(temps)[1]
    angles = zeros(Float64,len)
    if fonction == "polynome"
        angle_function = find_angle_polynome(angle, angle_echelon,0,10,1/3,1)
    else 
        angle_function = find_angle_spline(1,angle,angle_echelon,1)
    end

    for i=1:len
         angles[i] = angle_function(temps[i])
    end

    plot(temps,angles)
end
