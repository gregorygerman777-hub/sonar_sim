"""Record actual C++ sonar outputs with a scripted 3-D camera and sensor tour."""
from pathlib import Path
from datetime import datetime
import sys, json, subprocess
import numpy as np
import observatory_3d as app

out=app.ROOT/'results'/('demo_3d2d_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
out.mkdir(parents=True,exist_ok=False)
original_step=app.Lab.step
elapsed=0.
last_section=-1

def tour(self,dt):
    global elapsed,last_section
    elapsed+=dt
    section=min(2,int(elapsed/4))
    if section!=last_section:
        self.set_mode([1,3,2][section]);last_section=section
    local=elapsed-section*4
    self.orbit=-.9+.4*local
    self.elev=.48
    if section<2:self.position[0]=.45*np.sin(local*.7)
    else:self.roll=local*22.5
    original_step(self,dt)

app.Lab.step=tour
sys.argv=['record','--frames','360','--fixed-dt',str(1/30),'--record',str(out/'sonar_demo.gif'),'--screenshot',str(out/'final_frame.png')]
app.main()
import imageio_ffmpeg
subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-y','-i',str(out/'sonar_demo.gif'),
    '-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',str(out/'sonar_demo.mp4')],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
(out/'description.json').write_text(json.dumps({
    'duration_seconds':12,'fps':10,'resolution':[1120,700],
    'sections':['Sphere highlight and shadow with lateral sensor motion','Shared OBJ scene with camera orbit','Approximate surface multipath with 0–90 degree roll'],
    'rendering':'Pygame software 3-D projection and 2-D sonar fan, encoded with FFmpeg',
    'acoustics':'Existing C++ core via Cython; 97 bearings, 256 ranges, 256 elevation samples, top-hat, 600 kHz, four threads',
    'limitations':'Concept preview, not Unreal or calibrated DIDSON. Slow sweep is presentational. Camera and particles do not affect intensity.'},indent=2))
print('DEMO_OUTPUT='+str(out),flush=True)
