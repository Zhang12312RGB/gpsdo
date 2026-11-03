"""Generic SHA256 package audit. Run from any current directory."""
from pathlib import Path
import argparse,hashlib,json,re

def main():
    ap=argparse.ArgumentParser();ap.add_argument('root',nargs='?',type=Path,default=Path(__file__).resolve().parents[2]);a=ap.parse_args()
    root=a.root.resolve();manifest=json.loads((root/'manifest.json').read_text(encoding='utf-8'))
    failures=[]
    for r in manifest['files']:
        p=root/r['path']
        if not p.is_file(): failures.append('missing '+r['path']);continue
        if hashlib.sha256(p.read_bytes()).hexdigest()!=r['sha256']: failures.append('changed '+r['path'])
        for part in Path(r['path']).parts:
            if part not in ['README.md','LICENSE'] and not re.fullmatch(r'[a-z0-9_.-]+',part): failures.append('filename '+r['path'])
    print(json.dumps({'files':len(manifest['files']),'failures':failures,'result':'PASS' if not failures else 'FAIL'},ensure_ascii=False))
    if failures: raise SystemExit(1)

if __name__=='__main__':main()

