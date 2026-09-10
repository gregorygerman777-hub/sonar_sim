"""Execute in the selected interpreter and retain the partial notebook on failure."""
import os,sys
from pathlib import Path
import nbformat
from nbclient import NotebookClient
root=Path(__file__).resolve().parents[1]
output=Path(os.environ['SONAR_OUTPUT_DIR'])/'sonar_story.executed.ipynb'
notebook=nbformat.read(root/'notebooks/sonar_story.ipynb',as_version=4)
client=NotebookClient(notebook,timeout=900,resources={'metadata':{'path':str(root)}})
# KernelManager's default python3 kernelspec can point at another environment.
from jupyter_client import KernelManager
km=KernelManager();km.kernel_spec.argv=[sys.executable,'-m','ipykernel_launcher','-f','{connection_file}']
client.km=km
try:
    client.execute()
finally:
    nbformat.write(notebook,output)
print(f'Notebook PASS: {len(notebook.cells)} cells; {output}')
