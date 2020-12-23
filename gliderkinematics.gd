extends Node

var mpilot  = 90   # kg (including harness and clothing = Hook-in weight)
var h       = 1.2  # in m (hang strap length)

var mwing   = 22   # kg (mass of wing)
var Cmo     = 0.05
var I       = 102  # in kg.m**2 according to G.V , I=110;% according to Cook study. Inertia Pilot+ Glider about CGTotal

var AR      = 5.5  # Aspect ratio Falcon 5.5
var K       = 0.7  # Cmq variable (from Methods for Estimating Stability and Control Derivatives for Standard Subsonic Airplanes (1973) -Roskam p51  )

var S       = 15.8 # m^2 Falcon 170 area
var c       = 1.9  # m Falcon 170
var tpdist  = 8.5*c/100 # en m (distance between CGW et Tether point)
var cgdist  = 0.06 # distance between tether point and the CGW
var Clwa    = 3.5  # spanwise average value of the wing section lift curve slope (slope of CL between 5 and 15 degrees of alpha)
var cbar    = 1.55 # distance between the apex of the downtubes and the control bar axis (optional)
		
var Scx     = 0.16 # S.Cx of the pilot position. If prone : Scx=0.16. If stood: Scx=0.66
var g       = 9.81 # N/kg or 
var rho     = 1.22 # kg/m^3 density of air

var M       = mpilot + mwing
var phi     = deg2rad(14)   # angle in degrees between the downtube and the axis perpendicular to the keel
var sweep   = deg2rad(31)   # Sweep angle at the quarterchord


var u0      = 0             # initial position along flight path 
var v0      = 9             # initial airspeed in m/s
var alphr0  = deg2rad(20)   # initial angle of attack (alph=a+f)
var ar0     = deg2rad(-10)  # flight path angle in radians Gamma
var fr0     = alphr0 + ar0  # pitch attitude in radians/sec Theta
var br0     = deg2rad(0)    # Initial pitch rate attitude in radians/sec^2

# all points are relative to the aerodynamic centre
# y = ([0, 0, u, fr, v, ar, br])
#          u    position along flight path 
#          fr   pitch attitude in radians Theta ]
#          v    airspeed in m/s
#          ar   flight path angle in radians
#          br   pitch rate attitude in radians/sec^2
#          Lb   pilot position
var LbLo    = 0.15
var LbHi    = 0.74


func flightmotions(y, Lb):
	var u  = y[2]
	var fr = y[3]
	var v  = y[4]
	var ar = y[5]
	var br = y[6]

	var alpha = fr - ar
	var alphasq = alpha*alpha
	var alphacu = alpha*alphasq 

	var Clift  = -16.6*alphacu + 11.48*alphasq + 1.3*alpha + 0.038
	var Cdg    = 7.07*alphacu - 4.68*alphasq + 1.1*alpha - 0.0144

	var vsq    = v*v                  # airspeed square
	var lift   = 0.5*rho*vsq*S*Clift
	var Dcdg   = 0.5*rho*vsq*Cdg*S     # Drag of the wing alone
	var Dpilot = 0.5*rho*vsq*Scx       # Drag of the pilot alone
	var drag   = Dcdg + Dpilot         # Drag of the system (wing + pilot)

	var dyn    = 0.5*rho*vsq*S         # dynamic pressure

	var TP     = tpdist*Vector2(cos(fr), sin(fr))         # Tether point
	var CGW    = TP + cgdist*Vector2(cos(fr), sin(fr))    # CG of wing
	var ksi  = atan(Lb/h)                                 # angle between cg pilot and the downtubes
	var CGP  = TP + h*Vector2(sin(fr+phi+ksi), -cos(fr+phi+ksi))  # CG of pilot
	var CGT  = (CGP*mpilot + CGW*mwing)/(mpilot + mwing)  # Position CG of the system (pilot+wing)
	CGpilot = CGP

	# Tau=z[4];
	var Cx  = -Dcdg
	var Cy  = lift
	var d   = CGT.length()
	
	var Xw  = CGT.x
	var ARcu = AR*AR*AR
	var tansweep = tan(sweep)
	var Cmq = -K*Clwa*cos(sweep)*((((1/24)*(ARcu*tansweep*tansweep)/(AR + 6*cos(sweep))) + 1/8) + (AR*(2*(Xw/c) + 0.5*(Xw/c))/(AR+2*cos(sweep))))

	# Damping
	var Mq  = (Cmq*br*c*c*rho*v*S)/4
	var Mq2 = -0.5*rho*Cdg*S*(-2*br*d*d*v + br*br*d*d*d) 

	var XWT = CGW.x - CGT.x
	var XP  = CGP.x - CGT.x
	var YP  = CGP.y - CGT.y

	# Differential equations
	var dy = [0,0,0,0,0,0,0]
	dy[0] = v*cos(ar)    # change in x
	dy[1] = v*sin(ar)    # change in altitude
	dy[2] = v            # airspeed is velocity 
	dy[3] = br           # rate of change in pitch

	dy[4] = -g*sin(ar) - (drag/M)  # acceleration in direction of flight
	dy[5] = (1/v*(-g*cos(ar) + (lift/M)))  # angular change of direction of flight path angle
	#print(I, Cmo*dyn*c + mwing*g*XWT + mpilot*g*XP - Cy*CGT[0] - Cx*(-CGT[1]) - YP*Dpilot + Mq + Mq2)
	dy[6] = (Cmo*dyn*c + mwing*g*XWT + mpilot*g*XP - Cy*CGT[0] - Cx*(-CGT[1]) - YP*Dpilot + Mq + Mq2)/I
			# pitch rate attitude rate of change
	#dy[6] = 0
	return dy

var Y = [0, 0, u0, fr0, v0, ar0, br0]
var pos = Vector3(0,0,0)
var CGpilot = Vector3(0,-h,0)
func resetinitialconditions():
	Y = [0, 0, u0, fr0, v0, ar0, br0]

func stepdt(dt, Lb, hvec):
	var dY = flightmotions(Y, Lb)
	for i in range(len(Y)):
		Y[i] += dY[i]*dt
	pos = pos + Vector3(dY[0]*hvec.x*dt, dY[1]*dt, dY[0]*hvec.z*dt)
	return pos
