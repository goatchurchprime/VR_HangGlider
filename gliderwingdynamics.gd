extends Node

var S       = 13.7				# m^2 Puma area
var stall	= deg2rad(11)		#Stall angle
var c       = 1				# m Falcon 170
var Cmo     = 0.05
var Scx     = 0.16				# S.Cx of the pilot position. If prone : Scx=0.16. If stood: Scx=0.66
var g       = 9.81				# N/kg 
var rho     = 1.22				# kg/m^3 density of air

var alphavaluesaved = 0.0

# Note: Z+ points along the nose of the glider, and Y+ is the kingpost
# airvelocity is incoming towards the nose, 
# therefore to the first approximation it is (0, 1, 10) m/s
func linearforce(airvelocity):
	var Clift 	= 0
	var Cdg		= 0
	var alpha = -atan2(-airvelocity.y, -airvelocity.z)
	
	alpha = clamp(alpha, deg2rad(-35), deg2rad(35))
	alphavaluesaved = alpha
	var alphasq  = alpha*alpha
	var alphacu  = alpha*alphasq 

	if alpha < stall:
		Clift    = -0.75715*alphacu + 0.48844*alphasq + 4.7*alpha + 0.1
		Cdg      = -0.3*alphacu + 0.73656*alphasq - 0.02*alpha + 0.03
	else:
		Clift    = -6.87*alphasq + 7.44*alpha - 0.161
		Cdg      = 0.516*alphasq - 0.0314*alpha + 0.0261

	# resolution of the forces
	var airspeed = clamp(airvelocity.length(), 0, 30)
	var vsq      = airspeed*airspeed
	var lift     = 0.5*rho*vsq*S*Clift
	var Dcdg     = 0.5*rho*vsq*Cdg*S
	
	var avecDrag = airvelocity.normalized()
	var avecLift = Vector3(1, 0, 0).cross(avecDrag)

	return avecLift*lift + avecDrag*Dcdg
		
var Dcount = 0
func turningforce(airvelocity, wfrot, linearforce, CGtoACvec, hvec):
	var Clift 	= 0
	var Cdg		= 0
	var airspeed = clamp(airvelocity.length(), 0, 30)
	var vsq    = airspeed*airspeed
	var br     = clamp(-wfrot.x, -50, 50)
		
	var dyn    = 0.5*rho*vsq*S         # dynamic pressure
	var Xw     = -hvec.dot(CGtoACvec)

	var Cmq  = -0.2

	# Damping due to relative camber and larger displacement of wing chords due to wing sweep
	var Mq     = (Cmq*br*c*c*rho*airspeed*S)/4
	
	# Damping due to rotation of wing around the low centre of gravity
	var alpha  = -atan2(-airvelocity.y, -airvelocity.z)
	alpha      = clamp(alpha, deg2rad(-35), deg2rad(35))
	var alphasq= alpha*alpha
	var alphacu = alpha*alphasq 
	if alpha < stall:
		Clift    = -0.75715*alphacu + 0.48844*alphasq + 4.7*alpha + 0.1
		Cdg      = -0.3*alphacu + 0.73656*alphasq - 0.02*alpha + 0.03
	else:
		Clift    = -6.87*alphasq + 7.44*alpha - 0.161
		Cdg      = 0.516*alphasq - 0.0314*alpha + 0.0261

	var d       = CGtoACvec.length()
	var Mq2     = -0.5*rho*Cdg*S*(-2*br*d*d*airspeed + br*br*d*d*d)

	var dampingforceX = (Mq + Mq2)

	var rollangle      = atan2(airvelocity.x, airvelocity.y)
	var antirolltorque = 802*clamp(-rollangle*50, -50, 50)
	var rolldamptorque = -46000*wfrot.z*abs(wfrot.z)
		
	var yawangle      = atan2(-airvelocity.x, -airvelocity.z)
	var antiyawtorque = 42*clamp(-yawangle*50, -50, 50)
	var yawdamptorque = -4600*wfrot.y*abs(wfrot.y)
	
	Dcount += 1
	if Dcount == 800:
		print("yaw  ", yawangle, "  ", antiyawtorque, " ", yawdamptorque)
		Dcount = 0

	var torqueforceXwingforce = -CGtoACvec.cross(linearforce)
	var torqueforceX = Cmo*dyn*c
	var Dtw = torqueforceXwingforce.x + torqueforceX
	var Dtw1 = Dtw + dampingforceX
	return -torqueforceXwingforce + Vector3(-torqueforceX - dampingforceX, antiyawtorque + yawdamptorque, antirolltorque + rolldamptorque)
