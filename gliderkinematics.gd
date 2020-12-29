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

# values extracted from the AeroCentre model
var h       = 1.2  # in m (hang strap length)
var tpdist  = 8.5*c/100 # en m (distance between CGW et Tether point)
var cgdist  = 0.06 # distance between tether point and the CGW
var phi     = deg2rad(14)   # angle in degrees between the downtube and the axis perpendicular to the keel

var u0      = 0             # initial position along flight path 
var v0      = 9             # initial airspeed in m/s
var alphr0  = deg2rad(20)   # initial angle of attack (alph=a+f)
var ar0     = deg2rad(-10)  # flight path angle in radians Gamma
var fr0     = alphr0 + ar0  # pitch attitude in radians/sec Theta
var br0     = deg2rad(0)    # Initial pitch rate attitude in radians/sec^2

class GliderDynamicState:
	var v    # airspeed (m/s)
	var ar   # flight path angle (radians)
	var fr   # pitch attitude (radians)
	var br   # pitch rate attitude (radians/sec) (Dfr)

	# values calculated by flightforcesstate()
	var Dv   # rate of change of speed in flight direction (m/s^2)
	var Dar  # rate of change of flight path angle (radians/sec)
	var Dbr  # rate of change of pitch change velocity (radians/sec^2)

	func _init():
		v      = 9                # initial airspeed (m/s)
		var alphr0 = deg2rad(20)  # initial angle of attack (alph=a+f)
		ar     = deg2rad(-10)     # flight path angle (radians)
		fr     = alphr0 + ar      # pitch attitude (radians)
		br     = deg2rad(0)       # Initial pitch rate attitude (radians/sec)
	
	func stepflight(gliderpos, dt, hvec):
		var vh = v*cos(ar)
		var vv = v*sin(ar)
		var heading = 90
		var bbas = Basis(Vector3(1,0,0), -fr).rotated(Vector3(0,1,0), deg2rad(heading))
		var bpos = gliderpos.transform.origin + Vector3(vh*hvec.x, vv, vh*hvec.z)*dt
		gliderpos.transform = Transform(bbas, bpos)
		v += Dv*dt
		ar += Dar*dt
		fr += br*dt
		br += Dbr*dt
		
		var velocityvector = gliderpos.get_node("AeroCentre/TetherPoint/NosePoint/VelocityVector")
		velocityvector.rotation_degrees.x = rad2deg(fr - ar)
		velocityvector.scale.z = v
		gliderpos.get_node("AeroCentre/PitchRate").rotation_degrees.x = rad2deg(-br)



func _init(AeroCentre):
	h = AeroCentre.get_node("TetherPoint/HangStrap").mesh.size.y/2
	tpdist = AeroCentre.get_node("TetherPoint").transform.origin.z
	phi = deg2rad(-AeroCentre.get_node("TetherPoint/AframeBisector").rotation_degrees.x)
	cgdist = AeroCentre.get_node("TetherPoint/CGWing").transform.origin.z
	c = tpdist*100/8.5
	
func initgliderstate():
	return GliderDynamicState.new()

func flightforcesstate(s, Lb):
	var alpha = s.fr - s.ar
	var alphasq = alpha*alpha
	var alphacu = alpha*alphasq 

	var Clift  = -16.6*alphacu + 11.48*alphasq + 1.3*alpha + 0.038
	var Cdg    = 7.07*alphacu - 4.68*alphasq + 1.1*alpha - 0.0144

	var vsq    = s.v*s.v               # airspeed square
	var lift   = 0.5*rho*vsq*S*Clift
	var Dcdg   = 0.5*rho*vsq*Cdg*S     # Drag of the wing alone
	var Dpilot = 0.5*rho*vsq*Scx       # Drag of the pilot alone
	var drag   = Dcdg + Dpilot         # Drag of the system (wing + pilot)

	var dyn    = 0.5*rho*vsq*S         # dynamic pressure

	var TP     = tpdist*Vector2(cos(s.fr), sin(s.fr))         # Tether point
	var CGW    = TP + cgdist*Vector2(cos(s.fr), sin(s.fr))    # CG of wing
	var ksi  = atan(Lb/h)                                 # angle between cg pilot and the downtubes
	var CGP  = TP + h*Vector2(sin(s.fr+phi+ksi), -cos(s.fr+phi+ksi))  # CG of pilot
	var CGT  = (CGP*mpilot + CGW*mwing)/(mpilot + mwing)  # Position CG of the system (pilot+wing)

	# Tau=z[4];
	var Cx  = -Dcdg
	var Cy  = lift
	var d   = CGT.length()
	
	var Xw  = CGT.x
	var ARcu = AR*AR*AR
	var tansweep = tan(sweep)
	var Cmq = -K*Clwa*cos(sweep)*((((1/24)*(ARcu*tansweep*tansweep)/(AR + 6*cos(sweep))) + 1/8) + (AR*(2*(Xw/c) + 0.5*(Xw/c))/(AR+2*cos(sweep))))

	# Damping
	var Mq  = (Cmq*s.br*c*c*rho*s.v*S)/4
	var Mq2 = -0.5*rho*Cdg*S*(-2*s.br*d*d*s.v + s.br*s.br*d*d*d) 

	var XWT = CGW.x - CGT.x
	var XP  = CGP.x - CGT.x
	var YP  = CGP.y - CGT.y

	s.Dv = -g*sin(s.ar) - (drag/M)  # acceleration in direction of flight
	s.Dar = (1/s.v*(-g*cos(s.ar) + (lift/M)))  # angular change of direction of flight path angle
	s.Dbr = (Cmo*dyn*c + mwing*g*XWT + mpilot*g*XP - Cy*CGT[0] - Cx*(-CGT[1]) - YP*Dpilot + Mq + Mq2)/I

