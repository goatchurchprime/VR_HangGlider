extends Spatial

func glidergo():
	print("glider go!")
	$AnimationPlayer.play("circuit")

func _ready():
	$Plane.visible = false

