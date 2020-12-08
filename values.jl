# Constants of the aircraft
const physical_data = physic_cst()
const aircraft = avion()
const aircraft_cst = MiniBee_cst(aircraft)

# for resolution
const w,x,y,z = 1,0,0,0#angle2quaternion(0,[0,0,0])
println(w,x,y,z)
const v0x,v0y,v0z = 60,0,0
const m = aircraft.gross_mass
const X0 = @SVector [0,0,0,v0x,v0y,v0z,w,x,y,z,m]

const Tmax = 20
const T0 = 0.
const dt = 0.01
