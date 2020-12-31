extends Spatial


onready var pitchcontroller = get_node("../OQ_ARVROrigin/OQ_RightController")
onready var arvrorigin = get_node("../OQ_ARVROrigin")
onready var headcam = get_node("../OQ_ARVROrigin/OQ_ARVRCamera")
onready var label = get_node("../OQ_UILabel")
onready var orgpos = transform.origin
onready var windNoise = get_node("glider2/WindNoise3D")

var gliderkinematics = null
var gliderdynamicstate = null

func _ready():
	gliderkinematics = load("res://gliderkinematics.gd").new($AeroCentre)
	gliderdynamicstate = gliderkinematics.initgliderstate()
	gliderdynamicstate.stepflight(self, 0.0)
	windNoise.play()
		
var recarvrorigin = null
var headcamoffset = Vector3(0,0,0)
export var stationary = false

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
	
	var h = $AeroCentre/TetherPoint/HangStrap.mesh.size.y/2
	var Lb = clamp(-handdist, -h*0.9, h*0.9)
	var epsilon = rad2deg(asin(Lb/h))
	$AeroCentre/TetherPoint/HangStrap.rotation_degrees.x = -epsilon + $AeroCentre/TetherPoint/AframeBisector.rotation_degrees.x

	gliderkinematics.flightforcesstate(gliderdynamicstate, Lb, self)
	gliderdynamicstate.stepflight(self, delta)

		
	#Link wind noise volume and pitch to glider velocity
	#lb range = -0.6 (push out) to 0.2 (pull in)

	var windVolume =   gliderdynamicstate.v * 5 - 30 # 2.5 - 40
	windNoise.unit_db = windVolume 
	var windPitch = gliderdynamicstate.v * 0.08 -.1#-.4
	windNoise.pitch_scale = windPitch
	label.set_label_text("Lb=%f\nvolume=%f" % [Lb, rad2deg(windVolume)])
	
	if recarvrorigin == null:
		if abs(transform.origin.x - orgpos.x) > abs(orgpos.x)*2:
			transform.origin = orgpos
	elif abs(transform.origin.x - orgpos.x) > abs(orgpos.x)*20:
		transform.origin = orgpos
		
	#label.set_label_text("Lb=%f\npitch=%f" % [Lb, rad2deg(gliderdynamicstate.fr)])
	
	if pitchcontroller._button_just_pressed(vr.CONTROLLER_BUTTON.YB):
		if recarvrorigin == null:
			recarvrorigin = arvrorigin.global_transform.origin
			headcamoffset = headcam.global_transform.origin - arvrorigin.global_transform.origin
			#arvrorigin.global_transform.origin = $AeroCentre/TetherPoint/HangStrap/PilotBody/PilotHead.global_transform.origin - headcamoffset
		else:
			arvrorigin.global_transform.origin = recarvrorigin
			recarvrorigin = null
	if recarvrorigin != null:
		arvrorigin.global_transform.origin = $AeroCentre/TetherPoint/HangStrap/PilotBody/PilotHead.global_transform.origin - headcamoffset

