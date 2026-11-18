"""Build a collision world from the project's original occupancy grid, without editing it."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import yaml
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'simulation'
m=yaml.safe_load((ROOT/'maps/gallery.yaml').read_text())
a=np.asarray(Image.open(ROOT/'maps'/m['image']).convert('L'))
occ=(1-a/255)>m['occupied_thresh']
h,w=occ.shape; res=m['resolution']; ox,oy,yaw=m['origin']
sdf=ET.Element('sdf',version='1.9'); world=ET.SubElement(sdf,'world',name='arti_gallery')
def sub(parent,name,text,**attrs):
 e=ET.SubElement(parent,name,attrs); e.text=str(text); return e
p=ET.SubElement(world,'physics',name='default',type='ignored'); sub(p,'max_step_size',.01); sub(p,'real_time_factor',1)
for n,f in [('Physics','physics'),('UserCommands','user-commands'),('SceneBroadcaster','scene-broadcaster'),('Contact','contact')]:
 p=ET.SubElement(world,'plugin',name='gz::sim::systems::'+n,filename='gz-sim-'+f+'-system')
 if n=='Sensors':sub(p,'render_engine','ogre2')
gui=ET.SubElement(world,'gui'); camera=ET.SubElement(gui,'camera',name='arti_overview'); sub(camera,'pose','1 -0.2 4.5 0 0.916 2.289')
scene=ET.SubElement(world,'scene'); sub(scene,'ambient','0.65 0.65 0.65 1');sub(scene,'background','0.83 0.87 0.90 1')
l=ET.SubElement(world,'light',name='sun',type='directional');sub(l,'pose','0 0 10 0 0 0');sub(l,'diffuse','0.9 0.9 0.9 1');sub(l,'direction','-0.3 0.2 -1')
def box(name,x,y,z,sx,sy,sz,color,collision=True):
 model=ET.SubElement(world,'model',name=name);sub(model,'static','true');sub(model,'pose',f'{x} {y} {z} 0 0 {yaw}')
 link=ET.SubElement(model,'link',name='body')
 for kind in (['collision','visual'] if collision else ['visual']):
  node=ET.SubElement(link,kind,name=kind);geom=ET.SubElement(node,'geometry');b=ET.SubElement(geom,'box');sub(b,'size',f'{sx} {sy} {sz}')
  if kind=='visual':
   mat=ET.SubElement(node,'material');sub(mat,'ambient',color);sub(mat,'diffuse',color)
box('gallery_floor',ox+w*res/2,oy+h*res/2,-.05,w*res+2,h*res+2,.1,'0.82 0.78 0.69 1')
# Merge exactly matching horizontal runs across adjacent rows. No map dilation or shift.
active={};rect=[]
for r in range(h+1):
 runs=[]
 if r<h:
  starts=np.flatnonzero(np.diff(np.r_[False,occ[r],False].astype(int))==1)
  ends=np.flatnonzero(np.diff(np.r_[False,occ[r],False].astype(int))==-1)
  runs=list(zip(starts.tolist(),ends.tolist()))
 for key,begin in list(active.items()):
  if key not in runs:rect.append((*key,begin,r));del active[key]
 for key in runs:active.setdefault(key,r)
for i,(c0,c1,r0,r1) in enumerate(rect):
 x=(c0+c1)*res/2;y=(h-(r0+r1)/2)*res
 box(f'map_wall_{i}',ox+x*math.cos(yaw)-y*math.sin(yaw),oy+x*math.sin(yaw)+y*math.cos(yaw),.6,(c1-c0)*res,(r1-r0)*res,1.2,'0.72 0.76 0.80 1')
ET.indent(sdf);ET.ElementTree(sdf).write(OUT/'arti_gallery.sdf',encoding='unicode',xml_declaration=True)
print(f'{w}x{h}, {int(occ.sum())} occupied cells -> {len(rect)} wall boxes')
