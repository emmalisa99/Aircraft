# Constants of the aircraft
const physical_data = physic_cst()
const aircraft = avion()
const aircraft_cst = MiniBee_cst(aircraft)

# for resolution
const w,x,y,z = angle2quaternion(0,[0,1,0])
const v0x,v0y,v0z = 100,0,100
const X0 = @SVector [0,0,0,v0x,v0y,v0z,w,x,y,z,130000]

const Tmax = 10.
const T0 = 0.
const dt = 0.01
