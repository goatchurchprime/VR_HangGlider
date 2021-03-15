extends RigidBody

onready var pitchcontroller = get_node("../OQ_ARVROrigin/OQ_RightController")
onready var arvrorigin = get_node("../OQ_ARVROrigin")
onready var headcam = get_node("../OQ_ARVROrigin/OQ_ARVRCamera")
onready var label = get_node("../OQ_UILabel")

onready var labelvelocity = get_node("../OQ_UI2DCanvas").ui_control.get_node("airspeed")
onready var labelsinkrate = get_node("../OQ_UI2DCanvas").ui_control.get_node("sinkrate")
onready var labelangleofattack = get_node("../OQ_UI2DCanvas").ui_control.get_node("angleofattack")

onready var orgpos = transform.origin
onready var windNoise = get_node("glider2/WindNoise3D")
onready var gliderorigin = transform.origin
		
onready var gliderkinematics = $gliderkinematics
onready var gliderdynamicstate = $gliderdynamicstate
onready var gliderwingdynamics = $gliderkinematics/gliderwingdynamics

export var constraintopitch_andYZplane_only = true
#export var stationary = true

func takeoffstart():
	mode = RigidBody.MODE_KINEMATIC
	transform.origin = orgpos
	gliderkinematics.initgliderstate($AeroCentre)
	gliderdynamicstate.setgliderpos(self, transform.origin)
	windNoise.play()

func _ready():
	print($gliderkinematics.phi)
	takeoffstart()
	
var recarvrorigin = null
var headcamoffset = Vector3(0,0,0)


var altitudeshiftforPEcalc = 0

func _process(delta):
	if Input.is_action_just_pressed("ui_home") or pitchcontroller._button_just_pressed(vr.CONTROLLER_BUTTON.YB):
		if recarvrorigin == null:
			recarvrorigin = arvrorigin.global_transform.origin
			headcamoffset = headcam.global_transform.origin - arvrorigin.global_transform.origin
			#arvrorigin.global_transform.origin = $AeroCentre/TetherPoint/HangStrap/PilotBody/PilotHead.global_transform.origin - headcamoffset
		else:
			arvrorigin.global_transform.origin = recarvrorigin
			recarvrorigin = null

	if Input.is_action_just_pressed("ui_end"):
		windNoise.stop()
		mode = RigidBody.MODE_RIGID
		linear_velocity = gliderdynamicstate.vvec
			
	if Input.is_action_just_pressed("ui_page_down") or pitchcontroller._button_just_pressed(vr.CONTROLLER_BUTTON.XA):
		takeoffstart()

var energytimewindow = 0.2
var energytimer = 0.0
var prevtotalenergy = 0.0
var sumdragenergy = 0.0
var totalttime = 0
var Nintegralsubsteps = 10
func _physics_process(delta):
	var camvec = -headcam.global_transform.basis.z
	if is_nan(camvec.x):
		print("nan value in ARVRCamera!")
		return
	
	var controllerdisp = pitchcontroller.global_transform.origin - headcam.global_transform.origin
	var handdist = Vector3(camvec.x, 0, camvec.z).normalized().dot(controllerdisp)
	var handside = Vector3(-camvec.z, 0, camvec.x).normalized().dot(controllerdisp)
	if recarvrorigin != null:  # in flight hand disp needs to be in direction of flight or it responds to head turning
		var controlframeheading = $AeroCentre/TetherPoint/AframeBisector.global_transform.basis.z
		handdist = controlframeheading.dot(controllerdisp)
	
	var h = gliderkinematics.h
	var Lb = clamp(-handdist, -h*0.9, h*0.9)
	var epsilon = rad2deg(atan(Lb/h))   # probably should be asin
	var Ls = clamp(-handside, -h*0.6, h*0.6)
	var epsilonI = rad2deg(asin(Ls/h))
	$AeroCentre/TetherPoint/HangStrap.rotation_degrees.x = -epsilon + $AeroCentre/TetherPoint/AframeBisector.rotation_degrees.x
	$AeroCentre/TetherPoint/HangStrap.rotation_degrees.z = epsilonI
	

	if mode == RigidBody.MODE_KINEMATIC:
		var deltasubstep = delta/Nintegralsubsteps
		for i in Nintegralsubsteps:
			gliderkinematics.flightforcesstate(gliderdynamicstate, self)
			if constraintopitch_andYZplane_only:
				gliderdynamicstate.constraintoYZplane()
			gliderdynamicstate.stepflight(deltasubstep, self)
			sumdragenergy += deltasubstep*gliderdynamicstate.dragworkdone()

		transform.origin = gliderorigin + gliderdynamicstate.vvec*0.1 # Vector3(0, gliderdynamicstate.vvec.y*0.1, 0)
			
		energytimer += delta
		totalttime += delta
		if energytimer > energytimewindow:
			var potentialenergy = ($AeroCentre.global_transform.origin.y + altitudeshiftforPEcalc)*(gliderkinematics.mpilot + gliderkinematics.mwing)*gliderkinematics.g
			var kineticenergy = gliderdynamicstate.kineticenergy(gliderkinematics)
			var totalenergy = kineticenergy + potentialenergy
			#print(totalttime, "    ", gliderdynamicstate.vvec.length(), "  ", 0.5*(90+22)*gliderdynamicstate.vvec.length_squared(), "     ", kineticenergy)
			#print(totalttime, " ", totalenergy, " Wattage ", (totalenergy-prevtotalenergy)/energytimer, " dragWatt ", sumdragenergy/energytimer)
			var Drollangle = atan2(gliderdynamicstate.Dwingrelativeairspeedvector.x, gliderdynamicstate.Dwingrelativeairspeedvector.y)
			#print(totalttime, " r ", Drollangle, " av ", gliderdynamicstate.Dwingrelativeairspeedvector)
			#print(totalttime, " r ", Drollangle, gliderdynamicstate.Dfrot, gliderdynamicstate.frot)
			prevtotalenergy = totalenergy
			energytimer = 0.0

		var kinematiccollision = $KinematicBody.move_and_collide(gliderdynamicstate.vvec*delta, true, true, true)
		if kinematiccollision != null:
			windNoise.stop()
			mode = RigidBody.MODE_RIGID
			linear_velocity = gliderdynamicstate.vvec
			
	var windVolume = gliderdynamicstate.vvec.length() * 5 - 30 # 2.5 - 40
	windNoise.unit_db = windVolume 
	var windPitch = gliderdynamicstate.vvec.length() * 0.08 -.1#-.4
	windNoise.pitch_scale = windPitch
	
	label.set_label_text("Lb=%f\nvolume=%f" % [Lb, rad2deg(windVolume)])
	labelvelocity.text = "%.2f" % gliderdynamicstate.vvec.length()
	labelsinkrate.text = "%.2f" % (-gliderdynamicstate.vvec.y)
	labelsinkrate.text = "%.2f" % (-gliderdynamicstate.vvec.y)
	labelangleofattack.text = "%.1f" % rad2deg(gliderwingdynamics.alphavaluesaved)
	
	if mode == RigidBody.MODE_RIGID:
		pass
	elif recarvrorigin == null:
		if abs(transform.origin.x - orgpos.x) > abs(orgpos.x)*3:
			altitudeshiftforPEcalc += transform.origin.y - orgpos.y
			transform.origin = orgpos
	elif abs(transform.origin.x - orgpos.x) > abs(orgpos.x)*20:
		altitudeshiftforPEcalc += transform.origin.y - orgpos.y
		transform.origin = orgpos
		
	#label.set_label_text("Lb=%f\npitch=%f" % [Lb, rad2deg(gliderdynamicstate.fr)])
	
	if recarvrorigin != null:
		arvrorigin.global_transform.origin = $AeroCentre/TetherPoint/HangStrap/PilotBody/PilotHead.global_transform.origin - headcamoffset
