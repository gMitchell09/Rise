import bpy
import mathutils
from bpy.types import Window, Object
from mathutils import Vector
from math import sqrt, pi, atan2
from time import sleep, time
from array import array

import struct
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
    angleData = [atan2(face.normal.z, face.normal.y) for face in mesh.polygons]
    return angleData

def rotateLidar(object, angle):
    ori = object.rotation_euler
    ori.z += angle
    ori.z %= 360
    object.rotation_euler = ori
    #object.location.x += 0.001
    
    object.select = True
    bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
    
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

def duplicateObject(scene, name, copy_obj):
    
    mesh = bpy.data.meshes.new(name)
    ob_new = bpy.data.objects.new(name, mesh)
    ob_new.data = copy_obj.data.copy()
    ob_new.scale = copy_obj.scale
    ob_new.location = copy_obj.location
    ob_new.rotation_axis_angle = copy_obj.rotation_axis_angle   
    ob_new.parent = copy_obj.parent
    scene.objects.link(ob_new)
    
    return ob_new

def duplicateGroup(scene, group):
    for obj in group.objects:
        name = obj.name + "_tmp"
        duplicateObject(scene, name, obj)
    
def removeTempObjects(scene):
    for obj in scene.objects:
        if obj.name.find("_tmp") != -1:
            scene.objects.unlink(obj)
            
def moveRover(rover, distance):
    # assuming the rover is constrained to a path, we can move on any axis.
    rover.location.x += distance/3
    rover.location.y += distance/3
    rover.location.z += distance/3
    rover.select = True
    bpy.ops.object.transform_apply(location=True, rotation=False, scale=False)
    
def getObjectInLayer(name, layer):
    return [obj for obj in bpy.data.objects if obj.name.find(name) != -1 and obj.layers[layer] == True][0]
    
@oglWrapper
def main():
    
    # objects needed:
    #  - Lidar_Scan_Object + Mesh
    #  - IMU_Rotation
    #  - IMU_Position
    #  - Rover_Base
    
    # groups to copy:
    #  - lidarGroup, roverGroup
    
    print ("-----------------------Start----------------------")
    
    lidarGroup = bpy.data.groups["Lidar_Template"]
    roverGroup = bpy.data.groups["Rover_Template"]
    
    scene = bpy.data.scenes[0]
    
    #duplicateGroup(scene, lidarGroup)
    #duplicateGroup(scene, roverGroup)
    
    object = getObjectInLayer("Lidar_Scan_Object", 0) #bpy.data.objects['Lidar_Scan_Object']
    mesh = object.data
    imuPos = getObjectInLayer("IMU_Position", 0) #bpy.data.objects['IMU_Rotation']
    imuRot = getObjectInLayer("IMU_Rotation", 0) #bpy.data.objects['IMU_Position']
    
    roverPath = getObjectInLayer("RoverPath", 0) #bpy.data.objects["RoverPath"]
    roverBase = getObjectInLayer("Rover_Base", 0) #bpy.data.objects["Rover_Base_tmp"]
    
    #roverBase.constraints.new(type="CLAMP_TO")
    #roverBase.constraints["Clamp To"].target = roverPath
    #roverBase.constraints["Clamp To"].use_cyclic = True
    
    # /////////////////////////////////////////
    imu_file = createUniqueFile("imu", ".txt")
    lidar_file = createUniqueFile("lidar", ".txt")
    imu_pos_file = createUniqueFile("imu_pos", ".txt")
    
    print("Unique files created")
    
    imu_file_string = bytes()
    imu_pos_file_string = bytes()
    lidar_file_string = bytes()
    
    timestamp = 0 # ticks in ms
    startTime = current_milli_time()
    
    time_between_imu_readings = 5
    time_between_lidar_readings = 25
    
    rover_speed = 0.3 # mm/msec
    
    time_of_last_imu_read = -1
    time_of_last_imu_pos_read = -1
    time_of_last_lidar_read = -1
    
    imu_pos_start = imuPos.location

    # scan time = 25ms
    # rotation speed = 120RPM
    #                = 720 dps
    #                = 4 * pi rad/s
    #                = 18 d/iter
    #                = 0.15707963267948966192313216916398 rad/iter
    lidar_spin_speed = 0.01256637061435917295385057353312 # rad/ms

    while (timestamp < 2000):
        prev_timestamp = timestamp
        timestamp = timestamp + 1 #current_milli_time() - startTime
        
        if (timestamp - time_of_last_lidar_read > time_between_lidar_readings):
            dist_data = ScanOnce(object, mesh)
            longArray = array('L', [int(x) for x in dist_data])
            lidar_file_string += timestamp.to_bytes(4, sys.byteorder) + longArray.tobytes() + b'\0\0\0\0'
            #lidar_file_string += '~' + str(timestamp) + 'D' + ','.join(str(x) for x in dist_data) + 'E'
            time_of_last_lidar_read = timestamp
            
        if (timestamp - time_of_last_imu_read > time_between_imu_readings):
            quat = imuRot.matrix_world.to_quaternion()
            print ("Rot: ", imuRot.matrix_world.to_euler('XYZ'))
            floatArray = array('f', [quat.w, quat.x, quat.y, quat.z])
            imu_file_string += bytes('S', 'UTF-8') \
             + struct.pack('L', timestamp) \
             + bytes('TqL', 'UTF-8') \
             + struct.pack('b', len(floatArray) * 4) \
             + bytes('D', 'UTF-8') \
             + floatArray.tobytes() \
             + bytes('E', 'UTF-8')
            time_of_last_imu_read = timestamp
            
        if (timestamp - time_of_last_imu_pos_read > time_between_imu_readings):
            quat = imu_pos_start - imuPos.location
            #print ("Pos: ", imuPos.location)
            floatArray = array('f', [0, quat.x, quat.y, quat.z])
            imu_pos_file_string += bytes('S', 'UTF-8') \
             + struct.pack('L', timestamp) \
             + bytes('TqL', 'UTF-8') \
             + struct.pack('b', len(floatArray) * 4) \
             + bytes('D', 'UTF-8') \
             + floatArray.tobytes() \
             + bytes('E', 'UTF-8')
            time_of_last_imu_pos_read = timestamp
            
        rotateLidar(object, lidar_spin_speed * (timestamp - prev_timestamp))
        #moveRover(roverBase, rover_speed * (timestamp - prev_timestamp))
        #print ("Position: ", imuPos.location)
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
            
        #print("Time: ", timestamp)
        #print ("Q: ", imuObj.matrix_world.to_quaternion())
        
    bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
    sleep(1)
    removeTempObjects(scene)
    
    print ("")
    print ("")
    
    angleData = getFaceAngles(mesh)
    angleArray = array('f', angleData)
    lidar_file.write(angleArray)
    lidar_file.write(bytes('\x00\x00\x00\x00', 'UTF-8'))
    
    lidar_file.write(lidar_file_string)
    
    imu_timestamp_string = bytes('S\x00\x00\x00\x00TtL\x04D\x00\x00\x00\x00E', 'UTF-8')
    imu_file.write(imu_timestamp_string)
    imu_file.write(imu_file_string)
    imu_pos_file.write(imu_pos_file_string)
    
    lidar_file.close()
    imu_file.close()
    imu_pos_file.close()

main()