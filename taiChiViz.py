import json
import sys

import vtk
import time
import math

import pyaudio
import numpy as np
from itertools import cycle

class MyInteractorStyle(vtk.vtkInteractorStyleTrackballCamera):
    global ren
    def __init__(self,parent=None):
        self.parent = iren

        self.AddObserver("KeyPressEvent",self.keyPressEvent)

    def keyPressEvent(self,obj,event):
        key = self.parent.GetKeySym()
        print "IN KEYPRESS"
        if key == 'c':

            cam = vtk.vtkCamera()
            cam = ren.GetActiveCamera()
            pos = cam.GetPosition()
            at = cam.GetFocalPoint()
            up  = cam.GetViewUp()
            clip = cam.GetClippingRange()
            print "POS: " + str(pos[0]) + " " + str(pos[1]) + " " + str(pos[2])
            print "AT: " + str(at[0]) + " " + str(at[1]) + " " + str(at[2])
            print "UP: " + str(up[0]) + " " + str(up[1]) + " " + str(up[2])
            print "Clip: " + str(clip[0]) + " " + str(clip[1])

        if key == 'r':
            renWin.Render()


         

gscale = True
pos = []
at = []
up = []
clips = []
bounds = []
sphSize = 0
#set cmdline args/ shoudl check for existence
fileName = sys.argv[1];
stride = int(sys.argv[2])
trailSize = int(sys.argv[3])
pos.append(float(sys.argv[4]))
pos.append(float(sys.argv[5]))
pos.append(float(sys.argv[6]))
at.append(float(sys.argv[7]))
at.append(float(sys.argv[8]))
at.append(float(sys.argv[9]))
up.append(float(sys.argv[10]))
up.append(float(sys.argv[11]))
up.append(float(sys.argv[12]))
clips.append(float(sys.argv[13]))
clips.append(float(sys.argv[14]))
useGscale = int(sys.argv[15])
useIren = int(sys.argv[16])
sphSize = float(sys.argv[17])

if useGscale == 1:
    gscale = False

headCount = 0
headCurrentCount = 0
leftHandCount = 0
leftHandCurrentCount = 0
rightHandCount = 0
rightHandCurrentCount = 0
leftFootCount = 0
leftFootCurrentCount = 0
rightFootCount = 0
rightFootCurrentCount = 0
headTrajCount = 0
leftHandTrajCount = 0
rightHandTrajCount = 0
leftFootTrajCount = 0
rightFootTrajCount = 0
preReachedTrailSizeCount = 0
#xyz min and max for bounding box
xMin = 999999.0
xMax = -999999.0
yMin = 999999.0
yMax = -999999.0
zMin = 999999.0
zMax = -999999.0

#define lists for 3d coordinates for each sensor
Head=[]
LeftHand=[]
RightHand=[]
LeftFoot=[]
RightFoot=[]
#total velocity arrays
headTotalVel=[]
leftHandTotalVel=[]
rightHandTotalVel=[]
leftFootTotalVel=[]
rightFootTotalVel=[]

#allocate 3d points, this is making a list
#of xyz positions
ptsHead = vtk.vtkPoints()
ptsLH = vtk.vtkPoints()
ptsRH = vtk.vtkPoints()
ptsLF = vtk.vtkPoints()
ptsRF = vtk.vtkPoints()

#scalar array for opacity
scalars = vtk.vtkDoubleArray()
headTV = vtk.vtkDoubleArray()
leftHandTV = vtk.vtkDoubleArray()
rightHandTV = vtk.vtkDoubleArray()
leftFootTV = vtk.vtkDoubleArray()
rightFootTV = vtk.vtkDoubleArray()

source = vtk.vtkSphereSource()
headSource = vtk.vtkSphereSource()

#polydata objects for each sensor
#these represent the ability to have a geometric structure (connected)
#in our case a vertex at each 3d point
polyDataHead = vtk.vtkPolyData()
polyDataLH = vtk.vtkPolyData()
polyDataRH = vtk.vtkPolyData()
polyDataLF = vtk.vtkPolyData()
polyDataRF = vtk.vtkPolyData()


#appendData.UserManagedInputsOff()
#appendData.SetNumberOfInputs(5)
#3d glyph objects, class for dealing with solid geometries
glyphHead = vtk.vtkGlyph3D()
glyphLH = vtk.vtkGlyph3D()
glyphRH = vtk.vtkGlyph3D()
glyphLF = vtk.vtkGlyph3D()
glyphRF = vtk.vtkGlyph3D()


# depth sort objects so opacity works correctly
depthSortHead = vtk.vtkDepthSortPolyData()
depthSortLeftHand = vtk.vtkDepthSortPolyData()
depthSortRightHand = vtk.vtkDepthSortPolyData()
depthSortLeftFoot = vtk.vtkDepthSortPolyData()
depthSortRightFoot = vtk.vtkDepthSortPolyData()

headActor = vtk.vtkActor()
leftFootActor = vtk.vtkActor()
rightFootActor = vtk.vtkActor()
leftHandActor = vtk.vtkActor()
rightHandActor = vtk.vtkActor()

headMapper = vtk.vtkPolyDataMapper()
leftHandMapper = vtk.vtkPolyDataMapper()
rightHandMapper = vtk.vtkPolyDataMapper()
leftFootMapper = vtk.vtkPolyDataMapper()
rightFootMapper = vtk.vtkPolyDataMapper()

#lookup table for colors
lut = vtk.vtkLookupTable()

#boundingBox
bboxOutline = vtk.vtkOutlineSource()
bboxMapper = vtk.vtkPolyDataMapper()
bboxActor = vtk.vtkActor()

#image output
largeImage = vtk.vtkRenderLargeImage()
pngWriter = vtk.vtkPNGWriter()
# create a rendering window and renderer
ren = vtk.vtkRenderer()
#ren.SetBackground(1,1,1)
renWin = vtk.vtkRenderWindow()
renWin.AddRenderer(ren)

renWin.SetSize(2048,1536)
# create a renderwindowinteractor
iren = vtk.vtkRenderWindowInteractor()
iren.SetInteractorStyle(MyInteractorStyle())
iren.SetRenderWindow(renWin)
iren.Initialize()

#iren.AddObserver("KeyPressEvent",KeyPress)



def GetHeadData(d):
    global Head
    cnt = 0
    for dat in d['trackedDevice']['Head']['position']: 
        if cnt % stride == 0:
            Head.append([dat['x'],dat['y'],dat['z']])
        cnt += 1
       
def GetLeftFootData(d):
    global LeftFoot
    cnt = 0
    for dat in d['trackedDevice']['LeftFoot']['position']:
        if cnt % stride == 0:
            LeftFoot.append([dat['x'],dat['y'],dat['z']])
        cnt += 1
  
def GetRightFootData(d):
    global RightFoot
    cnt = 0
    for dat in d['trackedDevice']['RightFoot']['position']:
        if cnt % stride == 0:
            RightFoot.append([dat['x'],dat['y'],dat['z']])
        cnt += 1
  
def GetRightHandData(d):
    global RightHand
    cnt = 0
    for dat in d['trackedDevice']['RightHand']['position']:
        if cnt % stride == 0:
            RightHand.append([dat['x'],dat['y'],dat['z']])
        cnt += 1
  
def GetLeftHandData(d):
    global LeftHand
    cnt = 0
    for dat in d['trackedDevice']['LeftHand']['position']:
        if cnt % stride == 0:
            LeftHand.append([dat['x'],dat['y'],dat['z']])
        cnt += 1

def ComputeTotalVelocity():
    xPrev = yPrev = zPrev = xVel = yVel = zVel = 0;
    cnt = 0;
    for xyz in Head:
        if cnt > 0:
            xVel = (xyz[0]-xPrev)/2
            yVel = (xyz[1]-yPrev)/2
            zVel = (xyz[2]-zPrev)/2
            headTV.InsertNextValue(math.sqrt((xVel*xVel)+(yVel*yVel)+(zVel*zVel)));
            xPrev = xVel
            yPrev = yVel
            zPrev = zVel
        cnt += 1
    xPrev = yPrev = zPrev = xVel = yVel = zVel = 0;
    cnt = 0;
    for xyz in LeftHand:
        if cnt > 0:
            xVel = (xyz[0]-xPrev)/2
            yVel = (xyz[1]-yPrev)/2
            zVel = (xyz[2]-zPrev)/2
            leftHandTV.InsertNextValue(math.sqrt((xVel*xVel)+(yVel*yVel)+(zVel*zVel)));
            xPrev = xVel
            yPrev = yVel
            zPrev = zVel
        cnt += 1
    xPrev = yPrev = zPrev = xVel = yVel = zVel = 0;
    cnt = 0;
    for xyz in RightHand:
        if cnt > 0:
            xVel = (xyz[0]-xPrev)/2
            yVel = (xyz[1]-yPrev)/2
            zVel = (xyz[2]-zPrev)/2
            rightHandTV.InsertNextValue(math.sqrt((xVel*xVel)+(yVel*yVel)+(zVel*zVel)));
            xPrev = xVel
            yPrev = yVel
            zPrev = zVel
        cnt += 1
    xPrev = yPrev = zPrev = xVel = yVel = zVel = 0;
    cnt = 0;
    for xyz in LeftFoot:
        if cnt > 0:
            xVel = (xyz[0]-xPrev)/2
            yVel = (xyz[1]-yPrev)/2
            zVel = (xyz[2]-zPrev)/2
            leftFootTV.InsertNextValue(math.sqrt((xVel*xVel)+(yVel*yVel)+(zVel*zVel)));
            xPrev = xVel
            yPrev = yVel
            zPrev = zVel
        cnt += 1
    xPrev = yPrev = zPrev = xVel = yVel = zVel = 0;
    cnt = 0;
    for xyz in RightFoot:
        if cnt > 0:
            xVel = (xyz[0]-xPrev)/2
            yVel = (xyz[1]-yPrev)/2
            zVel = (xyz[2]-zPrev)/2
            rightFootTV.InsertNextValue(math.sqrt((xVel*xVel)+(yVel*yVel)+(zVel*zVel)));
            xPrev = xVel
            yPrev = yVel
            zPrev = zVel
        cnt += 1


def SetCamera():
    ren.GetActiveCamera().SetPosition(pos)
    ren.GetActiveCamera().SetFocalPoint(at)
    ren.GetActiveCamera().SetViewUp(up)
    ren.GetActiveCamera().SetClippingRange(clips)

def loadData(fname):
    with open(fname) as json_data:
        d = json.load(json_data)
        GetHeadData(d)
        GetLeftHandData(d)
        GetRightHandData(d)
        GetLeftFootData(d)
        GetRightFootData(d)

    setGlobalCounts()

def setGlobalCounts():
    global Head,LeftHand,LeftFoot,RightHand,RightFoot
    global headCount,leftHandCount,rightHandCount,leftFootCount,rightFootCount

    headCount = len(Head)
    leftHandCount = len(LeftHand)
    rightHandCount = len(RightHand)
    leftFootCount = len(LeftFoot)
    rightFootCount = len(RightFoot)

def Input3dPointsTraj():
    global headCurrentCount,leftHandCurrentCount,rightHandCurrentCount,leftFootCurrentCount,rightFootCurrentCount
    global headCount,leftHandCount,rightHandCount,leftFootCount,rightFootCount
    global headTrajCount,leftHandTrajCount,rightHandTrajCount,leftFootTrajCount,rightFootTrajCount
    global trailSize,preReachedTrailSizeCount
    global ptsHead,ptsLH,ptsRH,ptsLF,ptsRF
    global Head,LeftFoot,LeftHand,RightHand,RightFoot
    #points for Head
    ptsHead.Reset()
    ptsHead.InsertPoint(0,Head[headCurrentCount][0],Head[headCurrentCount][1],Head[headCurrentCount][2])
    headCurrentCount = headCurrentCount +1

    # if headTrajCount + trailSize < headCount:
    #     if(headCurrentCount < trailSize):
    #         for i in range(0,preReachedTrailSizeCount+1):
    #             ptsHead.InsertPoint(headCurrentCount,Head[headCurrentCount][0],Head[headCurrentCount][1],Head[headCurrentCount][2])
    #         if(headCurrentCount == trailSize):
    #             headTrajCount += 1
    #         else:
    #             headCurrentCount += 1
    #             preReachedTrailSizeCount = headCurrentCount;
    #     else:
    #         for i in range(headTrajCount,headTrajCount+trailSize):
    #             ptsHead.InsertPoint(i-headTrajCount,Head[i][0],Head[i][1],Head[i][2])
    #         headTrajCount += 1
    #points for leftHand
    ptsLH.Reset()
    preReachedTrailSizeCount = 0
    if leftHandTrajCount + trailSize < leftHandCount:
        if(leftHandCurrentCount < trailSize):
            for i in range(0,preReachedTrailSizeCount+1):
                ptsLH.InsertPoint(leftHandCurrentCount,LeftHand[leftHandCurrentCount][0],LeftHand[leftHandCurrentCount][1],LeftHand[leftHandCurrentCount][2])
            if leftHandCurrentCount == trailSize:
                leftHandTrajCount += 1
            else:
                leftHandCurrentCount += 1
                preReachedTrailSizeCount = leftFootCurrentCount
        else:
            for i in range(leftHandTrajCount,leftHandTrajCount+trailSize):
                ptsLH.InsertNextPoint(LeftHand[i][0],LeftHand[i][1],LeftHand[i][2])
            leftHandTrajCount += 1
    #points for rightHand
    ptsRH.Reset()
    preReachedTrailSizeCount = 0
    if rightHandTrajCount + trailSize < rightHandCount:
        if rightHandCurrentCount < trailSize:
            for i in range(0,preReachedTrailSizeCount+1):
                ptsRH.InsertPoint(rightHandCurrentCount,RightHand[rightHandCurrentCount][0],RightHand[rightHandCurrentCount][1],RightHand[rightHandCurrentCount][2])
            if rightHandCurrentCount == trailSize:
                rightHandTrajCount += 1
            else:
                rightHandCurrentCount += 1
                preReachedTrailSizeCount = rightHandCurrentCount
        else:
            for i in range(rightHandTrajCount,rightHandTrajCount+trailSize):
                ptsRH.InsertNextPoint(RightHand[i][0],RightHand[i][1],RightHand[i][2])
            rightHandTrajCount += 1
    #points for leftFoot
    ptsLF.Reset()
    preReachedTrailSizeCount = 0
    if leftFootTrajCount + trailSize < leftFootCount:
        if leftFootCurrentCount < trailSize:
            for i in range(0,preReachedTrailSizeCount+1):
                ptsLF.InsertPoint(leftFootCurrentCount,LeftFoot[leftFootCurrentCount][0],LeftFoot[leftFootCurrentCount][1],LeftFoot[leftFootCurrentCount][2])
            if leftFootCurrentCount == trailSize:
                leftFootTrajCount += 1
            else:
                leftFootCurrentCount += 1
                preReachedTrailSizeCount = leftFootCurrentCount
        else:
            for i in range(leftFootTrajCount,leftFootTrajCount+trailSize):
                ptsLF.InsertPoint(i-leftFootTrajCount,LeftFoot[i][0],LeftFoot[i][1],LeftFoot[i][2])
            leftFootTrajCount += 1
    #right foot points
    ptsRF.Reset()
    preReachedTrailSizeCount = 0
    if rightFootTrajCount + trailSize < rightFootCount:
        if rightFootCurrentCount < trailSize:
            for i in range(0,preReachedTrailSizeCount+1):
                ptsRF.InsertPoint(rightFootCurrentCount,RightFoot[rightFootCurrentCount][0],RightFoot[rightFootCurrentCount][1],RightFoot[rightFootCurrentCount][2])
            if rightFootCurrentCount == trailSize:
                rightFootTrajCount += 1
            else:
                rightFootCurrentCount += 1
                preReachedTrailSizeCount = rightFootCurrentCount
        else:
            for i in range(rightFootTrajCount,rightFootTrajCount+trailSize):
                ptsRF.InsertNextPoint(RightFoot[i][0],RightFoot[i][1],RightFoot[i][2])
            rightFootTrajCount += 1

def SetScalarData():
    global scalars,trailSize
    scalars.SetName("clr")
    for i in range(0,trailSize):
        scalars.InsertNextValue(float(i)/float(trailSize))


def MakeColorMap():
    lut.SetNumberOfTableValues(trailSize)
    lut.SetNumberOfColors(256)
    lut.SetHueRange(0.6667, 0.0)
    #lut.SetScaleToLog10()
    #lut.SetSaturationRange(1,1)
    #lut.SetValueRange(0.5,0.75)
    #lut.SetTableRange(1.75,2.2)
    #lut.SetAlphaRange(0.0,1.0)
    lut.Build()

def SetSphereSource():
    global source,sphSize
    source.SetRadius(sphSize);
    source.SetThetaResolution(20);
    source.SetPhiResolution(20);
    source.Update()

def SetSphereHeadSource():
    global headSource,sphSize
    headSource.SetRadius(sphSize*5);
    headSource.SetThetaResolution(20);
    headSource.SetPhiResolution(20);
    headSource.Update()

def SetPolyData():
    global polyDataHead,polyDataLH,polyDataRH,polyDataLF,polyDataRF
    global ptsLH,ptsRH,ptsLF,ptsRF,ptsHead
    polyDataHead.SetPoints(ptsHead)
    polyDataLH.SetPoints(ptsLH)
    polyDataRH.SetPoints(ptsRH)
    polyDataLF.SetPoints(ptsLF)
    polyDataRF.SetPoints(ptsRF)

    
    # polyDataHead.GetPointData().SetScalars(scalars)
    polyDataLH.GetPointData().SetScalars(scalars)
    polyDataRH.GetPointData().SetScalars(scalars)
    polyDataLF.GetPointData().SetScalars(scalars)
    polyDataRF.GetPointData().SetScalars(scalars)    
    

def SetGlyphs():
    global glyphHead,glyphLH,glyphRH,glyphLF,glyphRF
    global source, headSource

    glyphHead.SetSourceConnection(headSource.GetOutputPort()) #headSource
    glyphHead.SetInputData(polyDataHead)
    if gscale == False:
        glyphHead.ScalingOff()

    glyphLH.SetSourceConnection(source.GetOutputPort())
    glyphLH.SetInputData(polyDataLH)
    if gscale == False:
        glyphLH.ScalingOff()
    
    glyphRH.SetSourceConnection(source.GetOutputPort())
    glyphRH.SetInputData(polyDataRH)
    if gscale == False:
        glyphRH.ScalingOff()

    glyphLF.SetSourceConnection(source.GetOutputPort())
    glyphLF.SetInputData(polyDataLF)
    if gscale == False:
        glyphLF.ScalingOff()

    glyphRF.SetSourceConnection(source.GetOutputPort())
    glyphRF.SetInputData(polyDataRF)
    if gscale == False:
        glyphRF.ScalingOff()

def SetMappers():
    global glyphHead,glyphLH,glyphRH,glyphLF,glyphRF
    global headMapper,leftHandMapper,rightHandMapper,leftFootMapper,rightFootMapper,lut
    
    headMapper.SetInputData(depthSortHead.GetOutput())
    headMapper.SetLookupTable(lut)
    headMapper.MapScalars(0.25)
    
    leftHandMapper.SetInputData(depthSortLeftHand.GetOutput())
    leftHandMapper.SetLookupTable(lut)
    leftHandMapper.MapScalars(0.25)

    
    rightHandMapper.SetInputData(depthSortRightHand.GetOutput())
    rightHandMapper.SetLookupTable(lut)
    rightHandMapper.MapScalars(0.25)

    
    leftFootMapper.SetInputData(depthSortLeftFoot.GetOutput())
    leftFootMapper.SetLookupTable(lut)
    leftFootMapper.MapScalars(0.25)
    
    rightFootMapper.SetInputData(depthSortRightFoot.GetOutput())
    rightFootMapper.SetLookupTable(lut)
    rightFootMapper.MapScalars(0.25)
    

def SetActors():
    global headMapper,headActor,rightHandMapper,rightHandActor,leftHandMapper,leftHandActor
    global rightFootMapper,rightFootActor,leftFootMapper,leftFootActor,ren

    headActor.SetMapper(headMapper)
    ren.AddActor(headActor)
    leftHandActor.SetMapper(leftHandMapper)
    ren.AddActor(leftHandActor)
    rightHandActor.SetMapper(rightHandMapper)
    ren.AddActor(rightHandActor)
    leftFootActor.SetMapper(leftFootMapper)
    ren.AddActor(leftFootActor)
    rightFootActor.SetMapper(rightFootMapper)
    ren.AddActor(rightFootActor)
    
def DepthSortPolyDataGlyphs():
    global glyphHead,glyphLH,glyphRH,glyphLF,glyphRF
    ren.SetActiveCamera(ren.GetActiveCamera())
   
    depthSortHead.SetInputConnection(glyphHead.GetOutputPort())
    depthSortHead.SetDirectionToBackToFront()
    depthSortHead.SetVector(1,1,1)
    depthSortHead.SetCamera(ren.GetActiveCamera())
    depthSortHead.SortScalarsOff()
    depthSortHead.Update()

    depthSortLeftHand.SetInputConnection(glyphLH.GetOutputPort())
    depthSortLeftHand.SetDirectionToBackToFront()
    depthSortLeftHand.SetVector(1,1,1)
    depthSortLeftHand.SetCamera(ren.GetActiveCamera())
    depthSortLeftHand.SortScalarsOff()
    depthSortLeftHand.Update()

    depthSortRightHand.SetInputConnection(glyphRH.GetOutputPort())
    depthSortRightHand.SetDirectionToBackToFront()
    depthSortRightHand.SetVector(1,1,1)
    depthSortRightHand.SetCamera(ren.GetActiveCamera())
    depthSortRightHand.SortScalarsOff()
    depthSortRightHand.Update()

    depthSortLeftFoot.SetInputConnection(glyphLF.GetOutputPort())
    depthSortLeftFoot.SetDirectionToBackToFront()
    depthSortLeftFoot.SetVector(1,1,1)
    depthSortLeftFoot.SetCamera(ren.GetActiveCamera())
    depthSortLeftFoot.SortScalarsOff()
    depthSortLeftFoot.Update()

    depthSortRightFoot.SetInputConnection(glyphRF.GetOutputPort())
    depthSortRightFoot.SetDirectionToBackToFront()
    depthSortRightFoot.SetVector(1,1,1)
    depthSortRightFoot.SetCamera(ren.GetActiveCamera())
    depthSortRightFoot.SortScalarsOff()
    depthSortRightFoot.Update()


def RemoveActors():
    global headActor,leftHandActor,rightHandActor,leftFootActor,rightFootActor

    ren.RemoveActor(headActor)
    ren.RemoveActor(leftHandActor)
    ren.RemoveActor(rightHandActor)
    ren.RemoveActor(leftFootActor)
    ren.RemoveActor(rightFootActor)

def AddActors():
    global headActor,leftHandActor,rightHandActor,leftFootActor,rightFootActor

    ren.AddActor(headActor)
    ren.AddActor(leftHandActor)
    ren.AddActor(rightHandActor)
    ren.AddActor(leftFootActor)
    ren.AddActor(rightFootActor)

def GetMinAndMaxAllData():
    global xMin,xMax,yMin,yMax,zMin,zMax
    global Head,LeftHand,RightHand,LeftFoot,RightFoot
    for xyz in Head:
        if xyz[0] > xMax:
            xMax = xyz[0]
        if xyz[0] < xMin:
            xMin = xyz[0]
        if xyz[1] > yMax:
            yMax = xyz[1]
        if xyz[1] < yMin:
            yMin = xyz[1]
        if xyz[2] > zMax:
            zMax = xyz[2]
        if xyz[2] < zMin:
            zMin = xyz[2]
    for xyz in LeftHand:
        if xyz[0] > xMax:
            xMax = xyz[0]
        if xyz[0] < xMin:
            xMin = xyz[0]
        if xyz[1] > yMax:
            yMax = xyz[1]
        if xyz[1] < yMin:
            yMin = xyz[1]
        if xyz[2] > zMax:
            zMax = xyz[2]
        if xyz[2] < zMin:
            zMin = xyz[2]
    for xyz in RightHand:
        if xyz[0] > xMax:
            xMax = xyz[0]
        if xyz[0] < xMin:
            xMin = xyz[0]
        if xyz[1] > yMax:
            yMax = xyz[1]
        if xyz[1] < yMin:
            yMin = xyz[1]
        if xyz[2] > zMax:
            zMax = xyz[2]
        if xyz[2] < zMin:
            zMin = xyz[2]
    for xyz in LeftFoot:
        if xyz[0] > xMax:
            xMax = xyz[0]
        if xyz[0] < xMin:
            xMin = xyz[0]
        if xyz[1] > yMax:
            yMax = xyz[1]
        if xyz[1] < yMin:
            yMin = xyz[1]
        if xyz[2] > zMax:
            zMax = xyz[2]
        if xyz[2] < zMin:
            zMin = xyz[2]
    for xyz in RightFoot:
        if xyz[0] > xMax:
            xMax = xyz[0]
        if xyz[0] < xMin:
            xMin = xyz[0]
        if xyz[1] > yMax:
            yMax = xyz[1]
        if xyz[1] < yMin:
            yMin = xyz[1]
        if xyz[2] > zMax:
            zMax = xyz[2]
        if xyz[2] < zMin:
            zMin = xyz[2]

def SetBoundingBox():
    global xMin,xMax,yMin,yMax,zMin,zMax,ren
    global bboxOutline,bboxMapper,bboxActor
    
    bboxOutline.SetBounds(xMin,xMax,yMin,yMax,zMin,zMax)
    #bboxOutline.GenerateFacesOn()
    bboxMapper.SetInputConnection(bboxOutline.GetOutputPort())
    bboxActor.SetMapper(bboxMapper)
    bboxActor.GetProperty().SetColor(1.0,1.0,1.0)
    ren.AddActor(bboxActor)
    

def WritePngImage(fname):
    global largeImage,ren,pngWriter

    largeImage.SetInput(ren)
    largeImage.SetMagnification(1)
    pngWriter.SetInputConnection(largeImage.GetOutputPort())
    pngWriter.SetFileName(fname)
    pngWriter.Write()


loadData(fileName);

ComputeTotalVelocity()
SetScalarData()
MakeColorMap()

GetMinAndMaxAllData()
SetBoundingBox()
# this loop will create a trail of particles trailSize long for each point
# in the dataset modulus the stride

#iren.Initialize()




totalPoints = headCount/trailSize
print "TP: " + str(totalPoints) + " " + str(headCount)
if useIren == 0:
    fname = ""

    headColorIt = cycle(range(0, trailSize+1) + range(trailSize, 0, -1))

    for i in range(0,headCount):

        Input3dPointsTraj();
        SetSphereSource();
        SetPolyData();
        SetGlyphs();
        SetCamera()
        DepthSortPolyDataGlyphs()
        SetMappers()
        SetActors()
        
        headColor = next(headColorIt)
        headScalar = vtk.vtkDoubleArray()
        headScalar.InsertNextValue(scalars.GetValue(headColor))
        polyDataHead.GetPointData().SetScalars(headScalar)

        renWin.Render()
        SetSphereHeadSource();

        #time.sleep(0.01)
        #fname = "out_" + ("%04d" % i)+ ".png"
        #WritePngImage(fname)
        # print np.linalg.norm(np.array((LeftHand[i][0],LeftHand[i][1],LeftHand[i][2]))
        #     -np.array((RightHand[i][0],RightHand[i][1],RightHand[i][2])))


        RemoveActors()


else:
    # visualize a trail's worth then interact
    for i in range(0,trailSize):
        Input3dPointsTraj();
    SetSphereSource();
    SetPolyData();
    SetGlyphs();
    SetCamera()
    DepthSortPolyDataGlyphs()
    SetMappers()
    SetActors()
    renWin.Render()
    SetSphereHeadSource();
    
    iren.Start()
   
