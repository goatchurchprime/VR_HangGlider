tool
extends EditorScript

# *******
# Control-Shift X to run this code in the editor
# *******

class C:
	var frot = Vector3(0,0,0)
	var q
	func test(dt):
		var Dfquat = Quat(frot.normalized(), frot.length()*dt)
		print(Dfquat)

var c = C.new()
func _run():
	c.test(0.01667)
	print(sign(0))
	var q = Quat(Vector3(1,0,0), deg2rad(20))
	print(q)
	#var frot = Vector3(0,0,0)
	#var dt = 0.016667
	#var Dfquat = Quat(frot.normalized(), frot.length()*dt)
	#print(frot, Dfquat)
#		var Dfquat = Quat(frot.normalized(), frot.length()*dt)
	#var d = Quat(frot.normalized(), frot.length()*dt)
	#print(d)
