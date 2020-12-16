# Constants of the aircraft
const physical_data = physic_cst()
const aircraft = avion()
const aircraft_cst = MiniBee_cst(aircraft)

# for resolution
#const w,x,y,z = angle2quaternion(0,[0,0,0])
#println(w,x,y,z)
#const v0x,v0y,v0z = 10.,0.,0.
#const m = aircraft.gross_mass
#const X0 = @SVector [0,0,0,v0x,v0y,v0z,w,x,y,z,m]

# for solver resolution 
const Tmax = 10.
const T0 = 0.
const dt = 0.01

# Create the State
X0 = zeros(Float64,11)
position = [0,0,0]
X0[1:3] = position

const Angle = 5. #°  ## oritentation
w,x,y,z = angle2quaternion(Angle,[0,1,0])
X0[7:10] .= w,x,y,z

m = aircraft.gross_mass
g = physical_data.g
S = aircraft_cst.Sw
mass_vol = rho(X0[3], pressure, physical_data.r,physical_data.T)
coeff_aero = get_coeff(c_lift,Angle) 

v0x = sqrt(-2*m*g[3]/ (mass_vol * S * coeff_aero.Lift))
v0y = 0
v0z = 0
X0[4:6] .= v0x,v0y,v0z

X0[11] = m

# give the good thrust to keep the aircraft on the same level
function poussee()
    return 0.5 * v0x^2 * S * mass_vol * coeff_aero.Drag
end