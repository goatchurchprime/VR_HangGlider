extends RigidBody

onready var pitchcontroller = get_node("../OQ_ARVROrigin/OQ_RightController")
onready var arvrorigin = get_node("../OQ_ARVROrigin")
onready var headcam = get_node("../OQ_ARVROrigin/OQ_ARVRCamera")
onready var label = get_node("../OQ_UILabel")
onready var orgpos = transform.origin
onready var windNoise = get_node("glider2/WindNoise3D")

var gliderkinematics = null
var gliderdynamicstate = null

func takeoffstart():
	mode = RigidBody.MODE_KINEMATIC
	transform.origin = orgpos
	gliderdynamicstate = gliderkinematics.initgliderstate()
	gliderdynamicstate.setgliderpos(self, transform.origin)
	windNoise.play()

func _ready():
	gliderkinematics = load("res://gliderkinematics.gd").new($AeroCentre)
	takeoffstart()
	
var recarvrorigin = null
var headcamoffset = Vector3(0,0,0)
export var stationary = false

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

func _physics_process(delta):
	var camvec = -headcam.global_transform.basis.z
	if is_nan(camvec.x):
		print("nan value in ARVRCamera!")
		return
	
	var controllerdisp = pitchcontroller.global_transform.origin - headcam.global_transform.origin
	var handdist = Vector3(camvec.x, 0, camvec.z).normalized().dot(controllerdisp)	
	if recarvrorigin != null:  # in flight hand disp needs to be in direction of flight or it responds to head turning
		var controlframeheading = $AeroCentre/TetherPoint/AframeBisector.global_transform.basis.z
		handdist = controlframeheading.dot(controllerdisp)
	
	var h = gliderkinematics.h
	var Lb = clamp(-handdist, -h*0.9, h*0.9)
	var epsilon = rad2deg(atan(Lb/h))   # probably should be asin
	$AeroCentre/TetherPoint/HangStrap.rotation_degrees.x = -epsilon + $AeroCentre/TetherPoint/AframeBisector.rotation_degrees.x

	if mode == RigidBody.MODE_KINEMATIC:
		gliderkinematics.flightforcesstate(gliderdynamicstate, Lb, self)
		gliderdynamicstate.stepflight(delta, self)
		var kinematiccollision = $KinematicBody.move_and_collide(gliderdynamicstate.vvec*delta, true, true, true)
		if kinematiccollision != null:
			windNoise.stop()
			mode = RigidBody.MODE_RIGID
			linear_velocity = gliderdynamicstate.vvec
			
	#Link wind noise volume and pitch to glider velocity
	#lb range = -0.6 (push out) to 0.2 (pull in)

	var windVolume = gliderdynamicstate.vvec.length() * 5 - 30 # 2.5 - 40
	windNoise.unit_db = windVolume 
	var windPitch = gliderdynamicstate.vvec.length() * 0.08 -.1#-.4
	windNoise.pitch_scale = windPitch
	label.set_label_text("Lb=%f\nvolume=%f" % [Lb, rad2deg(windVolume)])
	
	if mode == RigidBody.MODE_RIGID:
		pass
	elif recarvrorigin == null:
		if abs(transform.origin.x - orgpos.x) > abs(orgpos.x)*3:
			transform.origin = orgpos
	elif abs(transform.origin.x - orgpos.x) > abs(orgpos.x)*20:
		transform.origin = orgpos
		
	#label.set_label_text("Lb=%f\npitch=%f" % [Lb, rad2deg(gliderdynamicstate.fr)])
	
	if recarvrorigin != null:
		arvrorigin.global_transform.origin = $AeroCentre/TetherPoint/HangStrap/PilotBody/PilotHead.global_transform.origin - headcamoffset
