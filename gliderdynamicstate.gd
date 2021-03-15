extends Node

# 3D interpretations of the state
#var fquat = Quat(-0.061628, 0.704416, 0.061628, 0.704416) # full glider attitude (derived from br)
# These numbers aren't used, they're reset in the init()!!!
var fquat = Quat(Vector3(1,0,0), deg2rad(20)) # full glider attitude (derived from br)
var vvec = Vector3(0, -2.083778, 11.817692)  # vector of flight
var frot = Vector3(0, 0, 0)   # rotation vector applied to the fquat

# values calculated by flightforcesstate()
var Dvvec = Vector3(0, 0, 0) # flight velocity acceleration (derived from Dv, Dar)
var Dfrot = Vector3(0, 0, 0)  # rate of rotation to add to the frot 

# conservation of energy check value
var dragforce = 0.0
var Dwingrelativeairspeedvector = Vector3(0, 0, 0)

func _init():
	var v  = 12               # initial airspeed (m/s)
	#v = 0
	var alphr0 = deg2rad(20)  # initial angle of attack (alph=a+f)
	var ar = deg2rad(-10)     # flight path angle (radians)
	var fr = alphr0 + ar      # pitch attitude (radians)
	var br = deg2rad(0)       # Initial pitch rate attitude (radians/sec)
	var attituderot = Vector3(1,0,0)
	var hquat   = Quat(Vector3(0,1,0), deg2rad(90))  # horizontal heading transform (z+ forward)

	hquat = Quat()
	fquat  = hquat*Quat(attituderot.normalized(), -fr)
	var hvec = hquat.xform(Vector3(0,0,1))
	vvec   = v*Vector3(cos(ar)*hvec.x, sin(ar), cos(ar)*hvec.z)
	frot   = Vector3(0.0, 0.0, br)

	print(fquat, vvec, frot)
	
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
	var NosePoint = gliderpos.get_node("AeroCentre/NosePoint")
	var VelocityVector = gliderpos.get_node("AeroCentre/NosePoint/VelocityVector")
	if vvec.x != 0 or vvec.z != 0:
		VelocityVector.global_transform = NosePoint.global_transform.looking_at(NosePoint.global_transform.origin+vvec, Vector3(0,1,0))
	VelocityVector.scale.z = vvec.length()
	gliderpos.get_node("AeroCentre/PitchRate").rotation_degrees.x = rad2deg(-frot.z)

func constraintoYZplane():
	Dvvec.x = 0
	Dfrot.y = 0
	Dfrot.z = 0
	vvec.x = 0  # unnecessary if already aligned
	fquat = Quat(fquat.x, 0, 0, fquat.w).normalized()  # unnecessary if already aligned

func kineticenergy(k):
	var linearenergy = 0.5*(k.mpilot + k.mwing)*vvec.length_squared()
	var angularenergy = 0.5*k.I*frot.length_squared()
	return linearenergy + 0*angularenergy
	
func dragworkdone():
	return dragforce*vvec.length() # + rotational drag?
