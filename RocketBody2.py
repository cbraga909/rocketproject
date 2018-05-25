from visual import *
from visual.graph import *

bodyLength=43
rocketRadius=2
coneLength=9.75
rootChord=6.75
tipChord=2.875
finHeight=2.25
noseRad=24

rocket=frame()
myboxes=frame()

xyplane=box(frame=myboxes, pos=(0,0,0), length=40, width= 0.1, height=40, color=color.cyan, opacity=0.2)
xzplane=box(frame=myboxes, pos=(0,0,0), length=40, width=40, height=0.1, color=color.green, opacity=0.2)
yzplane=box(frame=myboxes, pos=(0,0,0), length=0.1, width=40, height=40, color=color.magenta, opacity=0.2)

wall=shapes.rectangle(pos=vector(0,0,0),width=0.05, height=43)
circ=paths.circle(pos=vector(0,21.5), radius=2)
body=extrusion(frame=rocket,pos=circ,shape=wall,color=color.white)

FinShape=[(0,0),(0,rootChord),(finHeight,tipChord),(finHeight,0)]
Fin1=extrusion(frame=rocket,pos=[(2,0,0.125),(2,0,-0.125)],shape=Polygon(FinShape),color=color.blue)
Fin2=extrusion(frame=rocket,pos=[(-2,0,-0.125),(-2,0,0.125)],shape=Polygon(FinShape),color=color.red)
Fin3=extrusion(frame=rocket,pos=[(-0.125,0,2),(0.125,0,2)],shape=Polygon(FinShape),color=color.red)
Fin4=extrusion(frame=rocket,pos=[(0.125,0,-2),(-0.125,0,-2)],shape=Polygon(FinShape),color=color.red)
ar=shapes.arc(pos=vector(-23.5625,21.5),radius=23.5625,angle1=0,angle2=0.409)
NoseCone=extrusion(frame=rocket,pos=circ,shape=ar,color=color.red)





