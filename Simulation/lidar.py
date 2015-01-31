import bpy
import mathutils
from bpy.types import Window, Object
from mathutils import Vector
from math import sqrt, pi
from time import sleep, time
from array import array
import sys
import os

# line drawing stuffs
import bgl, blf

FT_TO_M = 0.3048
M_TO_FT = 1/FT_TO_M

def VectorMtF(vector):
    return vector * M_TO_FT

def VectorFtM(vector):
    return vector * FT_TO_M

def drawLine(start, end, color):
    bgl.glColor4f(*color)
    bgl.glBegin(bgl.GL_LINES)
    bgl.glVertex3f(*start)
    bgl.glVertex3f(*end)
    bgl.glEnd()

def ShootRay(start, end, center):
    # isCollision, obj, matrix, location, normal
    lineColor = [0, 0, 1.0, 1.0]
    drawLine(center, end, lineColor)
    col, obj, mat, loc, norm = bpy.data.scenes[0].ray_cast(center, end)
#    print ("Start: ", VectorMtF(center), "End: ", VectorMtF(end))
#    if (col):
#        print ("Object: ", obj)
#        print ("Loc: ", VectorMtF(loc))
    length = (loc - start).length
    return obj, col, length

def ScanOnce(object, mesh):
    #sleep(0.025)
    distData = []
    for face in mesh.polygons:
        #print ("Normal: ", str(face.normal))
        #print ("Object Center: ", VectorMtF(object.location))
        #print ("Face Center: ", VectorMtF(face.center + object.location))
        #maybe we need to xform into scene coordinates and then world?
        faceCenter = face.center + object.location
        # so our ray doesn't hit the face we are emitting the ray from, float variance
        faceOffset = 0.001 * face.normal
        obj, a, b = ShootRay(object.location, faceCenter + face.normal * float(30.0 * M_TO_FT), faceCenter + faceOffset);
        if (a):
            distData.append(b * 1000) # result in mm
            #print ("B: ", str(b * M_TO_FT))

        if (obj == object):
            print ("B: ", str(b * M_TO_FT))
            print ("Oh shit.")
            
    return distData

def getFaceAngles(mesh):
    angleData = [face.normal for face in mesh.polygons]          
    return angleData

def rotateLidar(object, angle):
    ori = object.rotation_euler
    ori.z += angle
    ori.z %= 360
    object.rotation_euler = ori
    #object.location.x += 0.001
    
    object.select = True
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    #bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
    
def correctLidarRotation(lidar, ref):
    print("Correcting rotation")

    ori = ref.rotation_euler.copy()
    print("Ori: ", ori)
    ori.z = -ori.z
    print("Inv: ", ori)

    lidar.rotation_euler = ori
    lidar.select = True
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)

def oglWrapper(func):
    def func_wrapper():
        bgl.glEnable(bgl.GL_BLEND)
        bgl.glColor4f(0.0, 0.0, 0.0, 0.5)
        bgl.glLineWidth(2)
        
        func()
                
        bgl.glLineWidth(1)
        bgl.glDisable(bgl.GL_BLEND)
        bgl.glColor4f(0.0, 0.0, 0.0, 1.0)    
    
    return func_wrapper

def createUniqueFile(name, ext):
    failedFiles = 0
    success = 0
    outFile = 0
    curPath = bpy.path.abspath("//")
    while (1):
        try:
            outFile = open(curPath + name + str(failedFiles) + ext, "x+b")
            break
        except:
            failedFiles += 1
            print ("Error: ", sys.exc_info()[0])
            continue
    return outFile

current_milli_time = lambda: int(round(time() * 1000))

@oglWrapper
def main():
    print ("-----------------------Start----------------------")
    mesh = bpy.data.meshes['Lidar_Scan_Mesh']
    object = bpy.data.objects['Lidar_Scan_Object']
    imuObj = bpy.data.objects['IMU_Object']
    rotObj = bpy.data.objects['Lidar_Rotation_Tracker']
    
    imu_file = createUniqueFile("imu", ".txt")
    lidar_file = createUniqueFile("lidar", ".txt")
    
    print("Unique files created")
    
    imu_file_string = bytes()
    lidar_file_string = ""
    
    timestamp = 0 # ticks in ms
    startTime = current_milli_time()
    
    time_between_imu_readings = 5
    time_between_lidar_readings = 25
    
    time_of_last_imu_read = -1
    time_of_last_lidar_read = -1

    # scan time = 25ms
    # rotation speed = 120RPM
    #                = 720 dps
    #                = 4 * pi rad/s
    #                = 18 d/iter
    #                = 0.15707963267948966192313216916398 rad/iter
    lidar_spin_speed = 0.01256637061435917295385057353312 # rad/ms
    
    while (timestamp < 20000):
        timestamp = current_milli_time() - startTime
        if (timestamp - time_of_last_lidar_read > time_between_lidar_readings):
            dist_data = ScanOnce(object, mesh)
            lidar_file_string += '~' + str(timestamp) + 'D' + ','.join(str(x) for x in dist_data) + 'E'
            time_of_last_lidar_read = timestamp
            
        if (timestamp - time_of_last_imu_read > time_between_imu_readings):
            quat = imuObj.matrix_world.to_quaternion()
            floatArray = array('f', [quat.w, quat.x, quat.y, quat.z])
            imu_file_string += bytes('~' + str(timestamp) + 'D', 'UTF-8') + floatArray.tobytes() + bytes('E', 'UTF-8')
            time_of_last_imu_read = timestamp
            
        rotateLidar(object, lidar_spin_speed * timestamp)
            
        #print("Time: ", timestamp)
        #print ("Q: ", imuObj.matrix_world.to_quaternion())
        
    bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
    sleep(5)
    correctLidarRotation(object, rotObj)
    
    print ("")
    print ("")
    
    lidar_file.write(bytes(lidar_file_string, 'UTF-8'))
    imu_file.write(imu_file_string)
    
    lidar_file.close()
    imu_file.close()

main()