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

onready var wingdynamics = $gliderwingdynamics


	
func initgliderstate(AeroCentre):
	h      = -AeroCentre.get_node("TetherPoint/HangStrap/PilotBody").transform.origin.y
	tpdist = AeroCentre.get_node("TetherPoint").transform.origin.z
	phi    = deg2rad(-AeroCentre.get_node("TetherPoint/AframeBisector").rotation_degrees.x)
	cgdist = AeroCentre.get_node("CGWing").transform.origin.z
	c      = tpdist*100/8.5
	wingdynamics.c = c


var Dcount = 0
func flightforcesstate(s, gliderpos):
	var ACpos  = gliderpos.get_node("AeroCentre").global_transform.origin
	var CGWpos = gliderpos.get_node("AeroCentre/CGWing").global_transform.origin
	var CGPpos = gliderpos.get_node("AeroCentre/TetherPoint/HangStrap/PilotBody").global_transform.origin
	var CGTotalpos = (CGPpos*mpilot + CGWpos*mwing)/(mpilot + mwing)

	var airspeedvector = localwindvector - s.vvec
	var wingrelativeairspeedvector = s.fquat.inverse().xform(airspeedvector)

	# new wing separated out calculations
	var wlinearforce = wingdynamics.linearforce(wingrelativeairspeedvector)
	var wfrot = s.fquat.inverse().xform(s.frot)
	var wCGtoACvec = s.fquat.inverse().xform(ACpos - CGTotalpos)
	
	var zvec = s.fquat.xform(Vector3(0,0,1))
	var hvec = s.fquat.inverse().xform(Vector3(zvec.x, 0, zvec.z).normalized())
	var Dxw = -hvec.dot(wCGtoACvec)
	
	var wturningforce = wingdynamics.turningforce(wingrelativeairspeedvector, wfrot, wlinearforce, wCGtoACvec, hvec)
	var linearforce = s.fquat.xform(wlinearforce)
	var turningforce = s.fquat.xform(wturningforce)

	var avecDrag = airspeedvector.normalized()
	s.Dwingrelativeairspeedvector = wingrelativeairspeedvector

	var airspeed = clamp(wingrelativeairspeedvector.length(), 0, 30)
	var vsq    = airspeed*airspeed     # airspeed square
	var Dpilot = 0.5*rho*vsq*Scx       # Drag of the pilot alone
	
	var dragPilot = Dpilot*avecDrag
	var torqueforceXpilotdrag = (CGPpos - CGTotalpos).cross(dragPilot)
	s.Dvvec = (linearforce + dragPilot)/M - Vector3(0, g, 0)
	s.dragforce = airspeedvector.normalized().dot(s.Dvvec)
	s.Dfrot = (turningforce + torqueforceXpilotdrag)/I
