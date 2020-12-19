using Polynomials
using DataInterpolations
using Dierckx
using Plots 
include("equations_equilibre.jl")


"""Modifier la valeur de l'angle voulu (en degré) et l'axe de rotation"""


# Attention ce calcul ne prend pas en compte un 
# cas plus complexe où l'axe d'inclinaison n'est pas quadratiq

function angle2quaternion(angle_voulu,axes,degre=true)
    if degre 
        angle_voulu = angle_voulu * pi / 180
    end
    nb_axes_concerne = sum(axes)
    xyz = sin( angle_voulu/2)
    w = sqrt(1 - xyz^2)
    angle_rotation = xyz * axes / nb_axes_concerne
    return w,angle_rotation[1],angle_rotation[2],angle_rotation[3]
end

function test_angle2quaternion(angle_deg)
    println("Angle en degré initial = 10")
    print("Quaternion obtenu : ")
    w,x,y,z = angle2quaternion(angle_deg,[0,1,0])
    println(w,x,y,z)
    println("Quaternion transformé en degré autour de [", x, ",",y,",",z ,"], est : ", 2 * acos(w) * 180 /pi, " ", 2 * asin(sqrt(x^2+y^2+z^2)) * 180 /pi)
end


####################################################################
## Find an equation that change the intial angle to an echelon angle
#################################################################### 
function find_angle_polynome(theta1,theta2,t,tf,x_pt,a_pt)
    d_angle = abs(theta1 - theta2)
    t = [t, t+tf, t+(tf-t)*x_pt]
    angle = [theta1, theta2, theta2+d_angle*a_pt]
    return fit(t,angle,2)
end

function find_angle_spline(d_temps,angle,angle_echelon,t_init)
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
