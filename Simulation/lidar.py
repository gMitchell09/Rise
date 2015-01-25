import bpy
import mathutils
from bpy.types import Window, Object
from mathutils import Vector
from math import sqrt

def ShootRay(start, end, center):
    # collision, obj, matrix, location, normal
    col, obj, mat, loc, norm = bpy.data.scenes[0].ray_cast(center, end)
    if (col):
        print ("Object: ", obj)
        print ("Mat: ", mat)
        print ("Loc: ", loc)
        print ("Norm: ", norm)
    length = (loc - start).length
    return col, length


print ("-----------------------Start----------------------")
#data = bpy.data.meshes['Lidar_Scan_Mesh']
#object = bpy.data.objects['Lidar_Scan_Object']
data = bpy.data.meshes['Cube']
object = bpy.data.objects['Cube']

for face in data.polygons:
    print ("Normal: ", str(face.normal))
    print ("Object Center: ", object.location)
    print ("Face Center: ", face.center)
    #maybe we need to xform into scene coordinates and then world?
    a, b = ShootRay(object.location, face.normal * float(1<<30), face.center);
    if (a):
        print ("B: ", str(b))
        
print ("")
print ("")
