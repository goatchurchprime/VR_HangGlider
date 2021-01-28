extends Node

var mpilot  = 90				# kg (including harness and clothing = Hook-in weight)
var mwing   = 22				# kg (mass of wing)
var M       = mpilot + mwing

var I       = 102				# in kg.m**2 according to G.V , I=110;% according to Cook study. Inertia Pilot+ Glider about CGTotal
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

# values extracted from the AeroCentre model
var h       = 1.2  # in m (hang strap length)
var tpdist  = 8.5*c/100 # en m (distance between CGW et Tether point)
var cgdist  = 0.06 # distance between tether point and the CGW
var phi     = deg2rad(14)   # angle in degrees between the downtube and the axis perpendicular to the keel

var localwindvector = Vector3(0, 0, 0)
const hquat   = Quat(Vector3(0,1,0), deg2rad(90))  # horizontal heading transform (z+ forward)

var useDvvec = true

class GliderDynamicState:
	var fr     # pitch attitude (radians) (visible in the 3D model)
	var v      # airspeed (m/s)
	var ar     # flight path angle (radians)
	var br     # pitch rate attitude (radians/sec) (Dfr)

	# 3D interpretations of the state
	var fquat  # full glider attitude (derived from br)
	var vvec   # vector of flight
	var frot   # rotation vector applied to the fquat
	
	# values calculated by flightforcesstate()
	var Dv     # rate of change of speed in flight direction (m/s^2)
	var Dar    # rate of change of flight path angle (radians/sec)
	var Dbr    # rate of change of pitch change velocity (radians/sec^2)
	var Dvvec  # flight velocity acceleration (derived from Dv, Dar)
	var Dfrot  # rate of rotation to add to the frot 
	
	var Dcount = 10
	
	func _init():
		v      = 12               # initial airspeed (m/s)
		var alphr0 = deg2rad(20)  # initial angle of attack (alph=a+f)
		ar     = deg2rad(-10)     # flight path angle (radians)
		fr     = alphr0 + ar      # pitch attitude (radians)
		br     = deg2rad(0)       # Initial pitch rate attitude (radians/sec)
		fquat = hquat*Quat(Vector3(1,0,0), -fr)
		print(fr, fquat*Vector3())
		var hvec = hquat.xform(Vector3(0,0,1))
		vvec = v*Vector3(cos(ar)*hvec.x, sin(ar), cos(ar)*hvec.z)
		frot = Vector3(0,0,br)
		
	func stepflight(dt, gliderpos):
		v = clamp(v + Dv*dt, 1, 50)
		ar = clamp(ar + Dar*dt, deg2rad(-35), deg2rad(35))
		fr = clamp(fr + br*dt,  deg2rad(-35), deg2rad(35))
		br = clamp(br + Dbr*dt, -10, 10)
		var hvec = hquat.xform(Vector3(0,0,1))
		vvec += Dvvec*dt
		vvec = v*Vector3(cos(ar)*hvec.x, sin(ar), cos(ar)*hvec.z)
		fquat = hquat*Quat(Vector3(1,0,0), -fr)
		setgliderpos(gliderpos, gliderpos.transform.origin + vvec*dt)

	func setgliderpos(gliderpos, bpos):
		gliderpos.transform = Transform(Basis(fquat), bpos)
		var VelocityVector = gliderpos.get_node("AeroCentre/TetherPoint/NosePoint/VelocityVector")
		VelocityVector.rotation_degrees.x = rad2deg(fr - ar)
		VelocityVector.scale.z = v
		gliderpos.get_node("AeroCentre/PitchRate").rotation_degrees.x = rad2deg(-br)


func _init(AeroCentre):
	h      = AeroCentre.get_node("TetherPoint/HangStrap").mesh.size.y/2
	var h1      = -AeroCentre.get_node("TetherPoint/HangStrap/PilotBody").transform.origin.y
	assert (h == h1)
	
	tpdist = AeroCentre.get_node("TetherPoint").transform.origin.z
	phi    = deg2rad(-AeroCentre.get_node("TetherPoint/AframeBisector").rotation_degrees.x)
	cgdist = AeroCentre.get_node("TetherPoint/CGWing").transform.origin.z
	c      = tpdist*100/8.5

func initgliderstate():
	return GliderDynamicState.new()

var Dcount = 10
# Y is up, X is direction of flight, Z is along the wing
func flightforcesstate(s, Lb, gliderpos):
	# angle of attack calculated from the airspeedvector and the full quaternion attitude
	var airspeedvector = localwindvector - s.vvec
	var wingrelativeairspeedvector = s.fquat.inverse().xform(airspeedvector)

	var alpha = -atan2(-wingrelativeairspeedvector.y, -wingrelativeairspeedvector.z)
	#   alpha = (s.fr - s.ar)
	alpha  = clamp(alpha, deg2rad(-35), deg2rad(35))

	var alphasq= alpha*alpha
	var alphacu= alpha*alphasq 
	var Clift  = -16.6*alphacu + 11.48*alphasq + 1.3*alpha + 0.038
	var Cdg    = 7.07*alphacu - 4.68*alphasq + 1.1*alpha - 0.0144
	assert (s.v>0)

	# resolution of the forces
	var airspeed = wingrelativeairspeedvector.length()
	var vsq    = airspeed*airspeed     # airspeed square
	var lift   = 0.5*rho*vsq*S*Clift
	var Dcdg   = 0.5*rho*vsq*Cdg*S     # Drag of the wing alone
	var Dpilot = 0.5*rho*vsq*Scx       # Drag of the pilot alone
	var drag   = Dcdg + Dpilot         # Drag of the system (wing + pilot)

	# step vector for position	
	s.Dv = -g*sin(s.ar) - drag/M  # acceleration in direction of flight
	s.Dar = 1/s.vvec.length()*(-g*cos(s.ar) + (lift/M))  # angular change of direction of flight path angle
	var avecLift = -gliderpos.get_node("AeroCentre").global_transform.basis.x.cross(s.vvec.normalized())

	# perhaps keep the forces separate so can use them in the turning moment equation
	s.Dvvec = avecLift*(lift/M) - s.vvec.normalized()*(drag/M) - Vector3(0, g, 0)

	
	# remaining calculation is of the torque on the glider
	# the turning moment is resolved about the CGTotal position

	var TP     = tpdist*Vector2(cos(s.fr), sin(s.fr))         # Tether point
	var CGW    = TP + cgdist*Vector2(cos(s.fr), sin(s.fr))    # CG of wing
	var TP3    = gliderpos.transform.basis.xform(gliderpos.get_node("AeroCentre/TetherPoint").transform.origin)
	var ksi    = atan(Lb/h)                                   # angle between cg pilot and the downtubes
	var CGP    = TP + h*Vector2(sin(s.fr+phi+ksi), -cos(s.fr+phi+ksi))  # CG of pilot
	var CGT    = (CGP*mpilot + CGW*mwing)/(mpilot + mwing)    # Position CG of the system (pilot+wing)

	# calculations directly from the oriented glider model to replace the above
	var ACpos  = gliderpos.get_node("AeroCentre").global_transform.origin
	var CGWpos = gliderpos.get_node("AeroCentre/TetherPoint/CGWing").global_transform.origin
	var CGPpos = gliderpos.get_node("AeroCentre/TetherPoint/HangStrap/PilotBody").global_transform.origin
	var CGTotalpos = (CGPpos*mpilot + CGWpos*mwing)/(mpilot + mwing)


	var dyn    = 0.5*rho*vsq*S         # dynamic pressure

	var Xw  = CGTotalpos.x - ACpos.x
	var Cmq = -K*Clwa*cossweep*((((1/24)*(ARcu*tansweep*tansweep)/(AR + 6*cossweep)) + 1/8) + (AR*(2*(Xw/c) + 0.5*(Xw/c))/(AR+2*cossweep)))

	# Damping due to relative camber and larger displacement of wing chords due to wing sweep
	var Mq  = (Cmq*s.br*c*c*rho*airspeed*S)/4
	
	# Damping due to rotation of wing around the low centre of gravity
	var d   = ACpos.distance_to(CGTotalpos)
	var Mq2 = -0.5*rho*Cdg*S*(-2*s.br*d*d*airspeed + s.br*s.br*d*d*d)

	var dampingforceX = Mq + Mq2

	# original matlab code
	var XP  = CGP.x - CGT.x
	var YP  = CGP.y - CGT.y
	var Cx  = -Dcdg
	var Cy  = lift
	var DtorqueforceXpilotdrag = -(CGPpos - CGTotalpos).y*Dpilot # looks too simplified as well
	var DtorqueforceXwingforce = -lift*(CGTotalpos - ACpos).x - Dcdg*(CGTotalpos - ACpos).y
	var DtorqueforceXweight = mwing*g*(CGWpos - CGTotalpos).x + mpilot*g*(CGPpos - CGTotalpos).x   # should be zero
	var DtorqueforceX = Cmo*dyn*c + DtorqueforceXweight + DtorqueforceXpilotdrag + DtorqueforceXwingforce
	#var torqueforceX = Cmo*dyn*c + mwing*g*(CGWpos - CGTotalpos).x + mpilot*g*(CGPpos - CGTotalpos).x - YP*Dpilot - Cy*CGT[0] - Cx*(-CGT[1])
	s.Dbr = (DtorqueforceX + dampingforceX)/I
	
	# version using cross products
	var avec = s.vvec.normalized()
	var wingforceAC = avecLift*lift - avec*Dcdg
	var torqueforceXwingforce = (ACpos - CGTotalpos).cross(wingforceAC)
	var torqueforceXpilotdrag = (CGPpos - CGTotalpos).cross(-avec*Dpilot)
	var torqueforceX = Vector3(0,0,Cmo*dyn*c) + torqueforceXpilotdrag + torqueforceXwingforce
	
	Dcount -= 1
	if Dcount == 0:
		#print( - Cy*CGT[0] - 0*Cx*(-CGT[1]), " ", torqueforceXwingforce)
		#print(torqueforceXweight, " ", torqueforceXwingforce, " ", torqueforceXwingforceD)
		#print(torqueforceXwingforce, " ", torqueforceXpilotdrag)
		print(torqueforceX, " ", DtorqueforceX)
		Dcount = 10

	s.Dfrot = (torqueforceX + Vector3(0,0,dampingforceX))/I
	s.Dfrot = Vector3(0, 0, s.Dbr)
	#torqueforceX = DtorqueforceX

	

