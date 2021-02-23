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

var wingdynamics = load("gliderwingdynamics.gd").new()

class GliderDynamicState:
	# 3D interpretations of the state
	var fquat  # full glider attitude (derived from br)
	var vvec   # vector of flight
	var frot   # rotation vector applied to the fquat
	
	# values calculated by flightforcesstate()
	var Dvvec  # flight velocity acceleration (derived from Dv, Dar)
	var Dfrot  # rate of rotation to add to the frot 
	
	# conservation of energy check value
	var dragforce
	var Dwingrelativeairspeedvector
	
	func _init():
		var v  = 12               # initial airspeed (m/s)
		#v = 0
		var alphr0 = deg2rad(20)  # initial angle of attack (alph=a+f)
		var ar = deg2rad(-10)     # flight path angle (radians)
		var fr = alphr0 + ar      # pitch attitude (radians)
		var br = deg2rad(0)       # Initial pitch rate attitude (radians/sec)
		var attituderot = Vector3(1,0,0)
		fquat  = hquat*Quat(attituderot.normalized(), -fr)
		var hvec = hquat.xform(Vector3(0,0,1))
		vvec   = v*Vector3(cos(ar)*hvec.x, sin(ar), cos(ar)*hvec.z)
		frot   = Vector3(0.1, 0, br)
		
	func stepflight(dt, gliderpos):
		frot += Dfrot*dt
		vvec += Dvvec*dt
		if frot != Vector3(0,0,0):
			var Dfquat = Quat(frot.normalized(), frot.length()*dt)
			#Dfquat = Quat(Vector3(0,0,1), frot.z*dt)
			fquat = Dfquat*fquat
		# clamp the rotation and the velocity
		if vvec.length() > 50:
			vvec *= 50/vvec.length()
		if frot.length() > 5:
			frot *= 5/frot.length()
		setgliderpos(gliderpos, gliderpos.transform.origin + vvec*dt)

	func setgliderpos(gliderpos, bpos):
		gliderpos.transform = Transform(Basis(fquat), bpos)
		assert (not (is_nan(gliderpos.transform.basis.x.x) or is_nan(gliderpos.transform.basis.x.y) or is_nan(gliderpos.transform.basis.x.z)))
		var NosePoint = gliderpos.get_node("AeroCentre/TetherPoint/NosePoint")
		var VelocityVector = gliderpos.get_node("AeroCentre/TetherPoint/NosePoint/VelocityVector")
		if vvec.x != 0 or vvec.z != 0:
			VelocityVector.global_transform = NosePoint.global_transform.looking_at(NosePoint.global_transform.origin+vvec, Vector3(0,1,0))
		VelocityVector.scale.z = vvec.length()
		gliderpos.get_node("AeroCentre/PitchRate").rotation_degrees.x = rad2deg(-frot.z)

	func kineticenergy(k):
		var linearenergy = 0.5*(k.mpilot + k.mwing)*vvec.length_squared()
		var angularenergy = 0.5*k.I*frot.length_squared()
		return linearenergy + 0*angularenergy
		
	func dragworkdone():
		return dragforce*vvec.length() # + rotational drag?

func _init(AeroCentre):
	h      = -AeroCentre.get_node("TetherPoint/HangStrap/PilotBody").transform.origin.y
	tpdist = AeroCentre.get_node("TetherPoint").transform.origin.z
	phi    = deg2rad(-AeroCentre.get_node("TetherPoint/AframeBisector").rotation_degrees.x)
	cgdist = AeroCentre.get_node("TetherPoint/CGWing").transform.origin.z
	c      = tpdist*100/8.5
	wingdynamics.c = c
	
func initgliderstate():
	return GliderDynamicState.new()

var Dcount = 0
# Y is up, X is direction of flight, Z is along the wing
func flightforcesstate(s, Lb, gliderpos):

	# calculations directly from the oriented glider model to replace the above
	var ACpos  = gliderpos.get_node("AeroCentre").global_transform.origin
	var CGWpos = gliderpos.get_node("AeroCentre/TetherPoint/CGWing").global_transform.origin
	var CGPpos = gliderpos.get_node("AeroCentre/TetherPoint/HangStrap/PilotBody").global_transform.origin
	var CGTotalpos = (CGPpos*mpilot + CGWpos*mwing)/(mpilot + mwing)

	var airspeedvector = localwindvector - s.vvec
	var wingrelativeairspeedvector = s.fquat.inverse().xform(airspeedvector)

	# new wing separated out calculations
	var wlinearforce = wingdynamics.linearforce(wingrelativeairspeedvector)
	var wfrot = s.fquat.inverse().xform(s.frot)
	var wCGtoACvec = s.fquat.inverse().xform(ACpos - CGTotalpos)
	
	var Xw  = CGTotalpos.x - ACpos.x
		# this VV expression doesn't handle negative values of Xw properly!
	var Dxw = Vector2(s.fquat.xform(wCGtoACvec).x, s.fquat.xform(wCGtoACvec).z).length()
	var wturningforce = wingdynamics.turningforce(wingrelativeairspeedvector, wfrot, wlinearforce, wCGtoACvec, Xw)
	var linearforce = s.fquat.xform(wlinearforce)
	var turningforce = s.fquat.xform(wturningforce)

	var avecDrag = airspeedvector.normalized()
	var avecWingAxis = gliderpos.get_node("AeroCentre").global_transform.basis.x
	var avecLift = avecWingAxis.cross(avecDrag)


	s.Dwingrelativeairspeedvector = wingrelativeairspeedvector
	var alpha = -atan2(-wingrelativeairspeedvector.y, -wingrelativeairspeedvector.z)

	var rollangle = atan2(wingrelativeairspeedvector.x, wingrelativeairspeedvector.y)

	var antirolltorque = s.fquat.xform(Vector3(0,0,402*clamp(-rollangle*50, -50, 50)))
	var rolldamptorque = s.fquat.xform(Vector3(0,0,-46000*wfrot.z*abs(wfrot.z)))

	alpha = clamp(alpha, deg2rad(-35), deg2rad(35))
	var alphasq= alpha*alpha
	var alphacu= alpha*alphasq 
	var Clift  = -16.6*alphacu + 11.48*alphasq + 1.3*alpha + 0.038
	var Cdg    = 7.07*alphacu - 4.68*alphasq + 1.1*alpha - 0.0144

	# resolution of the forces
	var airspeed = clamp(wingrelativeairspeedvector.length(), 0, 30)
	var vsq    = airspeed*airspeed     # airspeed square
	var lift   = 0.5*rho*vsq*S*Clift
	var Dcdg   = 0.5*rho*vsq*Cdg*S     # Drag of the wing alone
	var Dpilot = 0.5*rho*vsq*Scx       # Drag of the pilot alone
	var drag   = Dcdg + Dpilot         # Drag of the system (wing + pilot)

	# step vector for position
	s.Dvvec = avecLift*(lift/M) + avecDrag*(drag/M) - Vector3(0, g, 0)
	assert (not (is_nan(s.Dvvec.x) or is_nan(s.Dvvec.y) or is_nan(s.Dvvec.z)))
		
	# remaining calculation is of the torque on the glider
	# the turning moment is resolved about the CGTotal position
	#var br = clamp(s.frot.z, -50, 50)
	var br = s.frot.dot(-avecWingAxis)
	br = clamp(br, -50, 50)

	var dyn = 0.5*rho*vsq*S         # dynamic pressure
	var CmqA = (((1/24)*(ARcu*tansweep*tansweep)/(AR + 6*cossweep)) + 1/8)
	var CmqB = (AR*(2*(Xw/c) + 0.5*(Xw/c))/(AR+2*cossweep))
	var Cmq =-K*Clwa*cossweep*(CmqA + CmqB)

	# Damping due to relative camber and larger displacement of wing chords due to wing sweep
	var Mq  = (Cmq*br*c*c*rho*airspeed*S)/4
	
	# Damping due to rotation of wing around the low centre of gravity
	var d   = ACpos.distance_to(CGTotalpos)
	var Mq2 = -0.5*rho*Cdg*S*(-2*br*d*d*airspeed + br*br*d*d*d)

	var dampingforceX = -avecWingAxis*(Mq + Mq2)
	
	var wingforceAC = avecLift*lift + avecDrag*Dcdg
	var torqueforceXwingforce = (ACpos - CGTotalpos).cross(wingforceAC)
	var torqueforceXpilotdrag = (CGPpos - CGTotalpos).cross(avecDrag*Dpilot)
	torqueforceXpilotdrag = Vector3(0,0,0)
	var torqueforceXa = Cmo*dyn*c
	var torqueforceX = Vector3(0,0,torqueforceXa) + torqueforceXpilotdrag + torqueforceXwingforce

	# step vector for position
	s.Dfrot = (antirolltorque + rolldamptorque + torqueforceX + dampingforceX)/I
	s.dragforce = drag

	s.Dfrot = turningforce/I
	#s.Dvvec = (linearforce + dragPilot)/M - Vector3(0, g, 0)
		
	Dcount += 1
	if Dcount == 800:
		var dragPilot   = avecDrag*Dpilot
		var nDvvec = (linearforce + dragPilot)/M - Vector3(0, g, 0)
		print(s.Dvvec, nDvvec)
		var tdforceX = antirolltorque + rolldamptorque + torqueforceX + dampingforceX
		#print(tdforceX, turningforce, tdforceX.length(), " ", turningforce.length())
		# Xw = Dxw
		wingdynamics.turningforce(wingrelativeairspeedvector, wfrot, wlinearforce, wCGtoACvec, Xw)
		Dcount = 0


	

# Y is up, X is direction of flight, Z is along the wing
func flightforcesstateAntiRollOnly(s, Lb, gliderpos):
	var airspeedvector = localwindvector - s.vvec
	var avecDrag = airspeedvector.normalized()
	var avecWingAxis = gliderpos.get_node("AeroCentre").global_transform.basis.x
	var avecLift = avecWingAxis.cross(avecDrag)

	# printing this value feels like Z is direction of flight, Y is up and X is along the wing!
	var wingrelativeairspeedvector = s.fquat.inverse().xform(airspeedvector)
	s.Dwingrelativeairspeedvector = wingrelativeairspeedvector
	var alpha = -atan2(-wingrelativeairspeedvector.y, -wingrelativeairspeedvector.z)
	var rollangle = atan2(wingrelativeairspeedvector.x, wingrelativeairspeedvector.y)
	var antirolltorque = Vector3(102*clamp(-rollangle*50, -50, 50), 0, 0)
	var rolldamptorque = Vector3(-50*s.frot.x, 0, 0)
	s.Dvvec = Vector3(0,0,0)
	s.Dfrot = (antirolltorque + rolldamptorque)/I
	

