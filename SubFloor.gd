extends MeshInstance


var k = 0
func theight(x, y):
	return Vector3(x, -cos(x/50)*40 + sin((y+x)/10)*5 - x*0.2 + 60, y)

func _ready():
	var wx = mesh.size.x/2
	var wy = mesh.size.y/2
	var nx = 80
	var ny = 20
	 
	var mat = get_surface_material(0)
	var st = SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	for i in range(nx):
		var u0 = i*1.0/nx
		var u1 = (i+1)*1.0/nx
		var x0 = lerp(-wx, wx, u0)
		var x1 = lerp(-wx, wx, u1)
		for j in range(ny):
			var v0 = j*1.0/ny
			var v1 = (j+1)*1.0/ny
			var y0 = lerp(-wy, wy, v0)
			var y1 = lerp(-wy, wy, v1)
			
			st.add_uv(Vector2(u0, v0))
			st.add_vertex(theight(x0, y0))
			st.add_uv(Vector2(u1, v0))
			st.add_vertex(theight(x1, y0))
			st.add_uv(Vector2(u1, v1))
			st.add_vertex(theight(x1, y1))

			st.add_uv(Vector2(u0, v0))
			st.add_vertex(theight(x0, y0))
			st.add_uv(Vector2(u1, v1))
			st.add_vertex(theight(x1, y1))
			st.add_uv(Vector2(u0, v1))
			st.add_vertex(theight(x0, y1))
		
	st.generate_normals()
	mesh = st.commit()
	set_surface_material(0, mat)
