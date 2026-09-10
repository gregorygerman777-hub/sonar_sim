"""One recorded run: source identity, forced builds, tests, demos, notebook and UI."""
from pathlib import Path
from datetime import datetime
import os,sys,json,subprocess,time,hashlib,platform,re
root=Path(__file__).resolve().parents[1];os.chdir(root)
output=root/'results'/datetime.now().strftime('%Y-%m-%d_%H%M%S_%f');output.mkdir(parents=True,exist_ok=False)
env=dict(os.environ,SONAR_OUTPUT_DIR=str(output),MPLBACKEND='Agg',PYTHONUNBUFFERED='1')
manifest=dict(start=datetime.now().isoformat(),python=sys.version,executable=sys.executable,platform=platform.platform(),
    revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
    git_status=subprocess.check_output(['git','status','--short'],text=True),
    dependencies=subprocess.check_output([sys.executable,'-m','pip','freeze'],text=True),
    compiler=subprocess.check_output(['c++','--version'],text=True),
    parameters={'SONAR_MC_TRIALS':env.get('SONAR_MC_TRIALS','200'),'SONAR_MC_CONFIDENCE':env.get('SONAR_MC_CONFIDENCE','0.95')},
    seed_policy='Demo seeds are fixed in source; source hashes and full snapshot retained below; experiment JSON contains parameters.',stages=[],source_sha256={})
source_dir=output/'source';source_dir.mkdir()
for pattern in ('core/*','bindings/*.pyx','python/*.py','tests/*.py','notebooks/*.ipynb','assets/*','docs/*.md','setup.py','CMakeLists.txt','Makefile','run_all.sh','requirements.txt','README.md','LICENSE'):
    for p in root.glob(pattern):
        if p.is_file():
            relative=p.relative_to(root);manifest['source_sha256'][str(relative)]=hashlib.sha256(p.read_bytes()).hexdigest()
            target=source_dir/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(p.read_bytes())

def save():
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2))

def run(name,command,extra=None):
    print(f'[{name}] {" ".join(command)}',flush=True);start=time.perf_counter()
    path=output/(name+'.log')
    with path.open('w') as log:
        result=subprocess.run(command,env=dict(env,**(extra or {})),stdout=log,stderr=subprocess.STDOUT)
    manifest['stages'].append(dict(name=name,command=command,returncode=result.returncode,seconds=time.perf_counter()-start,log=path.name));save()
    print(path.read_text()[-1400:],flush=True)
    if result.returncode:raise RuntimeError(f'{name} failed ({result.returncode}); see {path}')

save()
try:
    run('cython_build',[sys.executable,'setup.py','build_ext','--inplace','--force'])
    run('import_identity',[sys.executable,'-c','import sonar; print(sonar.__file__)'])
    imported=(output/'import_identity.log').read_text().strip();manifest['extension_path']=imported
    manifest['extension_sha256']=hashlib.sha256(Path(imported).read_bytes()).hexdigest();save()
    run('cmake_configure',['cmake','-S','.', '-B',str(output/'cmake')])
    run('cmake_build',['cmake','--build',str(output/'cmake')])
    run('assets',[sys.executable,'python/mesh_assets.py'])
    run('unit_tests',[sys.executable,'tests/test_units.py'])
    run('research_tests',[sys.executable,'tests/test_research.py'])
    for demo in sorted((root/'python').glob('demo[0-9]*.py'),key=lambda p:int(re.search(r'demo(\d+)',p.name)[1])):
        run(demo.stem,[sys.executable,str(demo.relative_to(root))])
    run('benchmark',[sys.executable,'python/benchmark.py'])
    run('notebook',[sys.executable,'python/execute_notebook.py'])
    run('console_controls',[sys.executable,'python/console_check.py'])
    for size in ('1280x800','1920x1200'):
        run('pygame_'+size,[sys.executable,'python/research_console.py','--frames','3','--reconstruct','--size',size,'--screenshot',str(output/('console_'+size+'.png'))],dict(SDL_VIDEODRIVER='dummy',SDL_AUDIODRIVER='dummy'))
    run('pygame_animation',[sys.executable,'python/research_console.py','--frames','120','--animate','--size','1280x800','--screenshot',str(output/'console_animation.png')],dict(SDL_VIDEODRIVER='dummy',SDL_AUDIODRIVER='dummy'))
    manifest['status']='PASS'
except Exception as error:
    manifest['status']='FAIL';manifest['error']=repr(error);raise
finally:
    manifest['finish']=datetime.now().isoformat();save();print('RESULTS:',output,flush=True)
