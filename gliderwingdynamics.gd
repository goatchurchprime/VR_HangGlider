extends Node

var AR      = 5.5				# Aspect ratio Falcon 5.5
var K       = 0.7				# Cmq variable (from Methods for Estimating Stability and Control Derivatives for Standard Subsonic Airplanes (1973) -Roskam p51  )
var Cmo     = 0.05
var S       = 15.8				# m^2 Falcon 170 area
var c       = 1.9				# m Falcon 170
var sweep   = deg2rad(31)		# Sweep angle at the quarterchord
var Clwa    = 3.5				# spanwise average value of the wing section lift curve slope (slope of CL between 5 and 15 degrees of alpha)
var cbar    = 1.55				# distance between the apex of the downtubes and the control bar axis (optional)
		
var Scx     = 0.16				# S.Cx of the pilot position. If prone : Scx=0.16. If stood: Scx=0.66
var g       = 9.81				# N/kg 
var rho     = 1.22				# kg/m^3 density of air

# derived values
var ARcu    = AR*AR*AR
var tansweep= tan(sweep)
var cossweep= cos(sweep)

var alphavaluesaved = 0.0

# Note: Z+ points along the nose of the glider, and Y+ is the kingpost
# airvelocity is incoming towards the nose, 
# therefore to the first approximation it is (0, 1, 10) m/s
func linearforce(airvelocity):
	var alpha = -atan2(-airvelocity.y, -airvelocity.z)
	alphavaluesaved = alpha
	alpha = clamp(alpha, deg2rad(-35), deg2rad(35))
	var alphasq  = alpha*alpha
	var alphacu  = alpha*alphasq 
	#var Clift    = -16.6*alphacu + 11.48*alphasq + 1.3*alpha + 0.038
	#var Cdg      = 7.07*alphacu - 4.68*alphasq + 1.1*alpha - 0.0144
	var Clift    = -0.75715*alphacu + 0.48844*alphasq + 2.9383*alpha + 0.097839
	var Cdg      = -1.061965*alphacu + 0.73656*alphasq - 0.044846*alpha + 0.021052

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
	var airspeed = clamp(airvelocity.length(), 0, 30)
	var vsq    = airspeed*airspeed
	var br     = clamp(-wfrot.x, -50, 50)
		
	var dyn    = 0.5*rho*vsq*S         # dynamic pressure
	var Xw     = -hvec.dot(CGtoACvec)

	var CmqA = (((1.0/24)*(ARcu*tansweep*tansweep)/(AR + 6*cossweep)) + 1.0/8)
	var CmqB = (AR*(2*(Xw/c) + 0.5*(Xw/c))/(AR+2*cossweep))
	var Cmq  = -K*Clwa*cossweep*(CmqA + CmqB)

	# Damping due to relative camber and larger displacement of wing chords due to wing sweep
	var Mq     = (Cmq*br*c*c*rho*airspeed*S)/4
	
	# Damping due to rotation of wing around the low centre of gravity
	var alpha  = -atan2(-airvelocity.y, -airvelocity.z)
	alpha      = clamp(alpha, deg2rad(-35), deg2rad(35))
	var alphasq= alpha*alpha
	var alphacu = alpha*alphasq 
	#var Clift    = -16.6*alphacu + 11.48*alphasq + 1.3*alpha + 0.038
	#var Cdg      = 7.07*alphacu - 4.68*alphasq + 1.1*alpha - 0.0144
	var Clift    = -0.75715*alphacu + 0.48844*alphasq + 2.9383*alpha + 0.097839
	var Cdg      = -1.061965*alphacu + 0.73656*alphasq - 0.044846*alpha + 0.021052
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
