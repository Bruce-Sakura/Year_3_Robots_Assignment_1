import bpy, math, json, struct
from pathlib import Path
from mathutils import Vector
import xml.etree.ElementTree as E
P=Path(__file__).resolve().parent
D=P/'models'/'round_robot'; (D/'meshes').mkdir(parents=True,exist_ok=True)
bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete(use_global=False)
scene=bpy.context.scene; scene.unit_settings.system='METRIC'; scene.unit_settings.scale_length=1
parts=[]
def mat(name,col,metal=0):
 m=bpy.data.materials.new(name); m.diffuse_color=(*col,1); m.use_nodes=True
 bs=m.node_tree.nodes.get('Principled BSDF'); bs.inputs['Base Color'].default_value=(*col,1); bs.inputs['Metallic'].default_value=metal; bs.inputs['Roughness'].default_value=.24 if metal else .48
 return m
blue=mat('Blue chassis',(.025,.28,.65)); rubber=mat('Black rubber',(.035,.04,.045)); orange=mat('Orange hubs',(1,.34,.035)); silver=mat('Silver steel',(.72,.78,.85),.85); dark=mat('Caster housings',(.22,.26,.3),.6)
def finish(o,name,m,link='base_link'):
 o.name=name; o.data.materials.append(m); bpy.ops.object.transform_apply(location=False,rotation=True,scale=True); parts.append((o,link)); return o
def cylinder(name,loc,r,h,m,link='base_link',axis=False):
 bpy.ops.mesh.primitive_cylinder_add(vertices=96,radius=r,depth=h,location=loc,rotation=(math.pi/2,0,0) if axis else (0,0,0)); return finish(bpy.context.object,name,m,link)
def cube(loc,size):
 bpy.ops.mesh.primitive_cube_add(size=1,location=loc); o=bpy.context.object; o.dimensions=size; bpy.ops.object.transform_apply(location=False,rotation=False,scale=True); return o
def cut(o,c):
 mod=o.modifiers.new('Clearance','BOOLEAN'); mod.operation='DIFFERENCE'; mod.object=c; bpy.context.view_layer.objects.active=o; bpy.ops.object.modifier_apply(modifier=mod.name); bpy.data.objects.remove(c,do_unlink=True)
plate=cylinder('circular_chassis',(0,0,-.0125),.05,.005,blue)
for s in [-1,1]:cut(plate,cube((0,s*.061,-.0125),(.054,.038,.02)))
for s,label in [(1,'left'),(-1,'right')]:
 link=label+'_wheel'; cylinder(label+'_tire',(0,s*.05,0),.0242,.012,rubber,link,True)
 cylinder(label+'_hub',(0,s*.05,0),.014,.0124,orange,link,True)
 cylinder(label+'_axle',(0,s*.0365,0),.002,.015,silver,'base_link',True)
 bracket=finish(cube((0,s*.035,-.002),(.014,.004,.016)),label+'_vertical_axle_bracket',dark)
 bpy.ops.mesh.primitive_cylinder_add(vertices=64,radius=.0023,depth=.008,location=(0,s*.035,0),rotation=(math.pi/2,0,0)); cut(bracket,bpy.context.object)
 cylinder(label+'_motor',(0,s*.020,0),.006,.018,orange,'base_link',True)
 for x in [-.004,.004]:finish(cube((x,s*.020,-.007),(.003,.015,.006)),label+'_motor_foot_'+str(x),dark)


# Raised tread blocks stay inside the 25 mm effective rolling radius.
for side,label in [(1,'left'),(-1,'right')]:
 for n in range(48):
  a=n*math.tau/48
  for row in [-1,1]:
   block=cube((.02445*math.sin(a),side*.05+row*.0029,.02445*math.cos(a)),(.0015,.0052,.0009))
   block.rotation_euler.y=a
   finish(block,label+'_tread_'+str(n)+'_'+str(row),rubber,label+'_wheel')
# Elevated board; Pi 6 concept placeholder using Pi 5 envelope and hole locations.
green=mat('PCB green',(.025,.30,.085));gold=mat('Gold contacts',(.8,.56,.10),.65);red=mat('Power red',(.68,.018,.018));wireblack=mat('Wire black',(.016,.016,.016));yellow=mat('Control yellow',(.95,.68,.025));violet=mat('Control violet',(.34,.08,.65))
for x in [-.039,.019]:
 for y in [-.0245,.0245]:
  post=cylinder('pi_standoff_'+str(x)+'_'+str(y),(x,y,.004),.0025,.028,silver)
  bpy.ops.mesh.primitive_cylinder_add(vertices=32,radius=.00135,depth=.033,location=(x,y,.004));cut(post,bpy.context.object)
  cylinder('pi_screw_'+str(x)+'_'+str(y),(x,y,.0206),.0021,.0012,dark)
for y in [-.0245,.0245]:finish(cube((-.010,y,.016),(.063,.004,.004)),'pi_rack_rail_'+str(y),dark)
pcb=finish(cube((0,0,.0188),(.085,.056,.0016)),'Pi6_CONCEPT_PCB_using_Pi5_dimensions',green)
for x in [-.039,.019]:
 for y in [-.0245,.0245]:
  bpy.ops.mesh.primitive_cylinder_add(vertices=32,radius=.00135,depth=.006,location=(x,y,.0188));cut(pcb,bpy.context.object)
# Indicative ports and chips, all separate.
finish(cube((-.006,0,.0214),(.014,.014,.0036)),'pi_processor',dark)
finish(cube((.014,.003,.0210),(.010,.012,.0028)),'pi_memory',dark)
for y in [-.017,0,.017]:
 finish(cube((.036,y,.0258),(.013,.014,.0124)),'pi_port_shield_'+str(y),silver)
 finish(cube((.0428,y,.0258),(.0006,.010,.008)),'pi_port_opening_'+str(y),wireblack)
finish(cube((-.010,.023,.0221),(.052,.005,.005)),'pi_GPIO_header',wireblack)
for n in range(20):
 for yy in [.0217,.0243]:cylinder('gpio_pin_'+str(n)+'_'+str(yy),(-.034+n*.00254,yy,.025),.00035,.003,gold)
finish(cube((-.034,-.023,.0210),(.009,.009,.0028)),'pi_power_socket',silver)
# Driver board separates GPIO control from motor-power wiring.
finish(cube((.025,0,-.0075),(.018,.026,.0016)),'motor_driver_PCB',green)
finish(cube((.025,0,-.0050),(.010,.014,.0034)),'motor_driver_chip',dark)
for yy in [-.011,.011]:finish(cube((.025,yy,-.0038),(.014,.004,.006)),'driver_terminal_'+str(yy),blue)
def cable(name,points,material,r=.00055):
 cu=bpy.data.curves.new(name,'CURVE');cu.dimensions='3D';cu.resolution_u=16;cu.bevel_depth=r;cu.bevel_resolution=3
 sp=cu.splines.new('BEZIER');sp.bezier_points.add(len(points)-1)
 for b,p in zip(sp.bezier_points,points):b.co=p;b.handle_left_type='AUTO';b.handle_right_type='AUTO'
 o=bpy.data.objects.new(name,cu);scene.collection.objects.link(o);bpy.context.view_layer.objects.active=o;o.select_set(True)
 for obj in list(bpy.context.selected_objects):
  if obj!=o:obj.select_set(False)
 bpy.ops.object.convert(target='MESH');finish(bpy.context.object,name,material)
for side,label in [(1,'left'),(-1,'right')]:
 for xoff,material,suffix in [(-.0014,red,'plus'),(.0014,wireblack,'minus')]:
  cable(label+'_motor_wire_'+suffix,[(xoff,side*.011,0),(.010+xoff,side*.008,.003),(.020+xoff,side*.012,.003),(.025+xoff,side*.011,-.001)],material)
for yy,material,name in [(.0217,yellow,'PWM_left'),(.0243,violet,'PWM_right'),(.0229,wireblack,'common_ground')]:
 cable('GPIO_'+name,[(-.030,yy,.025),(-.043,yy,.027),(-.044,.010,.011),(-.030,-.006,-.003),(.025,-.009,-.001)],material,.0004)
for dx,material,name in [(0,red,'positive'),(.002,wireblack,'negative')]:
 cable('power_harness_'+name,[(-.034+dx,-.025,.021),(-.045+dx,-.026,.020),(-.044+dx,-.024,-.003),(-.025+dx,-.023,-.007),(.025+dx,0,-.002)],material)

for s,label in [(1,'front'),(-1,'rear')]:
 x=s*.036
 cup=cylinder(label+'_ball_socket',(x,0,-.01875),.008,.0075,dark)
 bpy.ops.mesh.primitive_uv_sphere_add(segments=48,ring_count=24,radius=.0053,location=(x,0,-.020)); cut(cup,bpy.context.object)
 bpy.ops.mesh.primitive_uv_sphere_add(segments=64,ring_count=32,radius=.005,location=(x,0,-.020)); ball=finish(bpy.context.object,label+'_steel_ball',silver,label+'_ball')
 for p in ball.data.polygons:p.use_smooth=True
# Both frames are literally the same point, at disk centre and axle midpoint.
for name in ['model_center','base_link']:
 o=bpy.data.objects.new(name,None); scene.collection.objects.link(o); o.empty_display_type='ARROWS'; o.empty_display_size=.025; o.location=(0,0,-.0125 if name=='model_center' else 0)
origins={'base_link':Vector((0,0,0)),'left_wheel':Vector((0,.05,0)),'right_wheel':Vector((0,-.05,0)),'front_ball':Vector((.036,0,-.020)),'rear_ball':Vector((-.036,0,-.020))}
# Export evaluated triangles with baked transforms and metre units. No joined objects.
records=[]
for o,link in parts:
 mesh=o.evaluated_get(bpy.context.evaluated_depsgraph_get()).to_mesh(); mesh.calc_loop_triangles()
 verts=[o.matrix_world@v.co-origins[link] for v in mesh.vertices]; tris=[tuple(t.vertices) for t in mesh.loop_triangles]
 name=o.name; rgb=tuple(o.data.materials[0].diffuse_color)
 with open(D/'meshes'/f'{name}.stl','wb') as f:
  f.write(b'Round robot SI metres'.ljust(80,b'\0')); f.write(struct.pack('<I',len(tris)))
  for tri in tris:
   a,b,c=[verts[i] for i in tri]; n=(b-a).cross(c-a).normalized(); f.write(struct.pack('<12fH',*n,*a,*b,*c,0))
 ns='http://www.collada.org/2005/11/COLLADASchema'; E.register_namespace('',ns)
 def tag(t):return '{'+ns+'}'+t
 def sub(p,t,txt=None,**a):
  q=E.SubElement(p,tag(t),a);q.text=txt;return q
 root=E.Element(tag('COLLADA'),version='1.4.1'); asset=sub(root,'asset'); sub(asset,'unit',name='meter',meter='1'); sub(asset,'up_axis','Z_UP')
 eff=sub(sub(root,'library_effects'),'effect',id='effect'); lam=sub(sub(sub(eff,'profile_COMMON'),'technique',sid='common'),'lambert');sub(sub(lam,'diffuse'),'color',' '.join(map(str,rgb)))
 ma=sub(sub(root,'library_materials'),'material',id='mat',name=name);sub(ma,'instance_effect',url='#effect')
 geo=sub(sub(root,'library_geometries'),'geometry',id='geo',name=name);me=sub(geo,'mesh')
 for sid,values in [('positions',[v for vec in verts for v in vec]),('normals',[v for tri in tris for v in ((verts[tri[1]]-verts[tri[0]]).cross(verts[tri[2]]-verts[tri[0]])).normalized()])]:
  src=sub(me,'source',id=sid);sub(src,'float_array',' '.join(f'{v:.9g}' for v in values),id=sid+'-array',count=str(len(values)));ac=sub(sub(src,'technique_common'),'accessor',source='#'+sid+'-array',count=str(len(values)//3),stride='3')
  for axis in 'XYZ':sub(ac,'param',name=axis,type='float')
 ve=sub(me,'vertices',id='verts');sub(ve,'input',semantic='POSITION',source='#positions')
 ts=sub(me,'triangles',count=str(len(tris)),material='surface');sub(ts,'input',semantic='VERTEX',source='#verts',offset='0');sub(ts,'input',semantic='NORMAL',source='#normals',offset='1');sub(ts,'p',' '.join(str(v) for n,tri in enumerate(tris) for i in tri for v in [i,n]))
 vs=sub(sub(root,'library_visual_scenes'),'visual_scene',id='Scene');node=sub(vs,'node',id=name,name=name);ig=sub(node,'instance_geometry',url='#geo');sub(sub(sub(ig,'bind_material'),'technique_common'),'instance_material',symbol='surface',target='#mat');sub(sub(root,'scene'),'instance_visual_scene',url='#Scene')
 E.indent(root);E.ElementTree(root).write(D/'meshes'/f'{name}.dae',encoding='utf-8',xml_declaration=True)
 records.append({'name':name,'link':link,'color':rgb})
(P/'parts.json').write_text(json.dumps(records,indent=2))
# Presentation camera, lights and ground; kept in a separate collection.
studio=bpy.data.collections.new('STUDIO - not exported');scene.collection.children.link(studio)
def move_studio(o):
 for c in list(o.users_collection):c.objects.unlink(o)
 studio.objects.link(o)
bpy.ops.mesh.primitive_plane_add(size=200,location=(0,0,-.0253));floor=bpy.context.object;floor.name='STUDIO_floor';floor.data.materials.append(mat('Floor',(.62,.65,.69)));move_studio(floor)
bpy.ops.object.camera_add(location=(.18,-.22,.18));cam=bpy.context.object;cam.rotation_euler=(Vector((0,0,-.005))-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.type='ORTHO';cam.data.ortho_scale=.23;scene.camera=cam;move_studio(cam)
for loc,power,size in [((.1,-.1,.25),.7,.18),((-.1,.15,.18),.5,.15)]:
 bpy.ops.object.light_add(type='AREA',location=loc);light=bpy.context.object;light.data.energy=power;light.data.shape='DISK';light.data.size=size;light.rotation_euler=(-light.location).to_track_quat('-Z','Y').to_euler();move_studio(light)
scene.world.color=(.25,.25,.25);scene.render.engine='CYCLES';scene.cycles.samples=32;scene.cycles.use_denoising=False;scene.render.resolution_x=1000;scene.render.resolution_y=850;scene.render.resolution_percentage=100
bpy.ops.wm.save_as_mainfile(filepath=str(P/'round_robot.blend'));scene.render.filepath=str(P/'preview.png');bpy.ops.render.render(write_still=True)
print('Built independent parts:',len(parts))
floor.hide_render=True
cam.location=(.18,-.22,-.16);cam.rotation_euler=(Vector((0,0,-.01))-cam.location).to_track_quat('-Z','Y').to_euler()
for o in studio.objects:
 if o.type=='LIGHT':o.location.z=-abs(o.location.z);o.rotation_euler=(-o.location).to_track_quat('-Z','Y').to_euler()
scene.render.filepath=str(P/'preview_underside.png');bpy.ops.render.render(write_still=True)
