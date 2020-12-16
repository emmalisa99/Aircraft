using Polynomials
using StaticArrays

struct Coeff
    Lift::Float64
    Drag::Float64
end

function Coeff(lift)
    C_D_min =  0.027 #0.031 #
    K = 0.07 # 1/(pi*aircraft_cst.e_os*aircraft_cst.AR)   # 0.054 #
    Lift = lift
    Drag = C_D_min + lift^2 * K
    Drag = 
    return Coeff(Lift,Drag)
end

function List_Lift(ListCoeff)
    n = size(ListCoeff)[1]
    Lift = zeros(Float64,n)
    for i=1:n
        Lift[i] = ListCoeff[i].Lift
    end
    return Lift
end

function List_Drag(ListCoeff)
    n = size(ListCoeff)[1]
    Drag = zeros(Float64,n)
    for i=1:n
        Drag[i] = ListCoeff[i].Drag
    end
    return Drag  
end

angle2coeff = Dict()
# angle2coeff[0.] = Coeff(0.)
# angle2coeff[1.] = Coeff(0.1)
# angle2coeff[2.] = Coeff(0.195)
# angle2coeff[3.] = Coeff(0.29)
# angle2coeff[4.] = Coeff(0.385)
# angle2coeff[5.] = Coeff(0.48)
# angle2coeff[6.] = Coeff(0.575)
# angle2coeff[7.] = Coeff(0.67)
# angle2coeff[8.] = Coeff(0.765)
# angle2coeff[9.] = Coeff(0.86)
# angle2coeff[10.] = Coeff(0.95)
# angle2coeff[11.] = Coeff(1.09)
# angle2coeff[12.] = Coeff(1.18)
# angle2coeff[13.] = Coeff(1.29)
# angle2coeff[14.] = Coeff(1.38)

# angle2coeff[-5.] = Coeff(-0.15)
# angle2coeff[-2.5] = Coeff(0.1)
# angle2coeff[0.] = Coeff(0.38)
# angle2coeff[2.5] = Coeff(0.55)
# angle2coeff[5.] = Coeff(0.75)
# angle2coeff[7.5] = Coeff(1.)
# angle2coeff[10.] = Coeff(1.22)
# angle2coeff[12] = Coeff(1.4)
# angle2coeff[13.5] = Coeff(1.51)
# angle2coeff[15.] = Coeff(1.6)


angle2coeff[2.] = Coeff(0.238)
angle2coeff[4.] = Coeff(0.451)
angle2coeff[6.] = Coeff(0.659)
angle2coeff[8.] = Coeff(0.855)
angle2coeff[10.] = Coeff(1.04)


angles = [collect(keys(angle2coeff))...]
coeff = collect(values(angle2coeff))
lift = List_Lift(coeff)
drag= List_Drag(coeff)

# give the linear function 
f_lift = fit(angles,lift,1)
# extract coefficient
c_lift = coeffs(f_lift)

# return the C_L according to the angle
function get_coeff(c_lift,angle)
    lift = c_lift[2] * angle + c_lift[1]
    lift = 0.55
    return Coeff(lift)
end

