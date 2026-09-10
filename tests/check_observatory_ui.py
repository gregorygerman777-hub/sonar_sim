import os,sys,json
from pathlib import Path
os.environ['SDL_VIDEODRIVER']='dummy';os.environ['SDL_AUDIODRIVER']='dummy'
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'python'))
import pygame,observatory_3d
original=pygame.event.get;frame=0

def events():
    global frame
    frame+=1;items=original()
    keys={1:pygame.K_SPACE,5:pygame.K_1,6:pygame.K_e,8:pygame.K_p}
    if frame in keys:items.append(pygame.event.Event(pygame.KEYDOWN,key=keys[frame]))
    # All positions use the actual 1600x1000 window, so no coordinate ambiguity.
    clicks={2:(160,820),3:(120,698),4:(120,646),7:(1460,610)}
    if frame in clicks:items.append(pygame.event.Event(pygame.MOUSEBUTTONDOWN,button=1,pos=clicks[frame]))
    return items
pygame.event.get=events
before=set((root/'results').glob('observatory_*'))
sys.argv=['check','--frames','9','--size','1600x1000','--screenshot',str(root/'output/observatory_controls.png')]
observatory_3d.main()
session=(set((root/'results').glob('observatory_*'))-before).pop()
report=json.loads((session/'session.json').read_text())
assert report['gain_db']==3
assert report['mode']==0
assert report['elevation_error']<1e-12
assert report['selected_bearing']!=48
assert list(session.glob('capture_*.png'))
print('PASS: real Pygame pause, gain, toggles, experiment selection, elevation flip, bearing selection and capture events')
