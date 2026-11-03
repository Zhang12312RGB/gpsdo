from pathlib import Path
import argparse,json,subprocess

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--gcc',default='gcc');a=ap.parse_args()
    here=Path(__file__).resolve().parent;src=here.parent/'src/sw';out=here/'generated'
    out.mkdir(exist_ok=True);result=[]
    slow=(src/'slow_loop.c').read_text(encoding='utf-8')
    core=slow.split('/* BEGIN LIMITED C WEIGHT CORE:',1)[1].split('/* END LIMITED C WEIGHT CORE',1)[0]
    core=core.split('*/',1)[1]
    (out/'weight_core.c').write_text('#include "slow_loop.h"\n#include <math.h>\n'+core,encoding='utf-8')
    for name in ['holdover','log','weight']:
        exe=out/('test_'+name+('.exe' if __import__('os').name=='nt' else ''))
        extra=[str(src/'kalman.c')] if name=='holdover' else ([str(out/'weight_core.c')] if name=='weight' else [])
        cmd=[a.gcc,'-std=c99','-Wall','-Wextra','-Wformat=2','-Werror','-I'+str(src),str(here/'host'/f'test_{name}.c'),*extra,'-lm','-o',str(exe)]
        p=subprocess.run(cmd,capture_output=True,text=True,errors='replace')
        if p.returncode: raise RuntimeError(p.stdout+p.stderr)
        p=subprocess.run([str(exe)],capture_output=True,text=True,errors='replace')
        if p.returncode: raise RuntimeError(p.stdout+p.stderr)
        print(p.stdout.strip());result.append({'test':name,'result':'PASS','output':p.stdout.strip()})
    (out/'host_results.json').write_text(json.dumps(result,indent=2),encoding='utf-8')

if __name__=='__main__':main()

