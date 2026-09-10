"""Exercise real Pygame events without needing a visible display."""
import os,sys,json
from pathlib import Path
os.environ.setdefault('SDL_VIDEODRIVER','dummy');os.environ.setdefault('SDL_AUDIODRIVER','dummy')
sys.path[:0]=['.','python']
import pygame
import research_console
original=pygame.event.get;tick=0
# Select sonar page, change display gain, capture, switch target, start/stop run.
def events():
    global tick
    result=original();tick+=1
    if tick in (2,3):result.append(pygame.event.Event(pygame.KEYDOWN,key=pygame.K_TAB))
    if tick==4:
        # Coordinates on the 1600x1000 canvas, gain slider on SONAR page.
        result.extend([pygame.event.Event(pygame.MOUSEBUTTONDOWN,pos=(1510,342),button=1),pygame.event.Event(pygame.MOUSEBUTTONUP,pos=(1510,342),button=1)])
    if tick==5:result.append(pygame.event.Event(pygame.KEYDOWN,key=pygame.K_p))
    if tick==6:result.append(pygame.event.Event(pygame.KEYDOWN,key=pygame.K_k))
    return result
pygame.event.get=events
sys.argv=['console_check','--frames','8','--size','1600x1000','--screenshot',str(Path(os.environ.get('SONAR_OUTPUT_DIR','output'))/'console_controls.png')]
before=set(Path('results').glob('console_*'))
research_console.main()
created=set(Path('results').glob('console_*'))-before
session=json.loads((created.pop()/'session.json').read_text())
assert session['pings']==1, f'Display-only control changed ping: {session}'
assert session['parameters']['gain dB']>10
print('PASS: gain drag, parameter tabs, capture and carving preserve cached acoustic ping.')
