"""Build frozen PS sources against the supplied AX7010 BSP. Python standard library."""
from pathlib import Path
import argparse, hashlib, json, subprocess

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--gcc',required=True,help='Vitis arm-none-eabi-gcc executable')
    ap.add_argument('--mode',type=int,choices=[0,2],default=2)
    ap.add_argument('--output',type=Path)
    a=ap.parse_args()
    root=Path(__file__).resolve().parent.parent
    src=root/'src/sw';bsp=root/'build/bsp'
    out=(a.output or root/'build/rebuilt'/f'mode{a.mode}').resolve()
    out.mkdir(parents=True,exist_ok=True)
    flags=['-DSDT','-mcpu=cortex-a9','-mfpu=vfpv3','-mfloat-abi=hard',
           '-specs='+str(bsp/'xilinx.spec'),'-I'+str(src),'-I'+str(bsp/'include'),
           f'-DEXP_SLOW_MODE={a.mode}']
    commands=[]
    def run(cmd):
        commands.append(cmd)
        p=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,errors='replace')
        log.write(p.stdout);log.flush()
        if p.returncode: raise RuntimeError('Build failed; see '+str(out/'build.log'))
    with (out/'build.log').open('w',encoding='utf-8') as log:
        objects=[]
        for f in sorted(src.glob('*.c')):
            obj=out/(f.stem+'.o');objects.append(str(obj))
            run([a.gcc,*flags,'-MMD','-MP','-c',str(f),'-o',str(obj)])
        elf=out/'gpsdo.elf'
        run([a.gcc,*flags,*objects,'-o',str(elf),'-Wl,-T,'+str(src/'lscript.ld'),
             '-L'+str(bsp/'lib'),'-Wl,--start-group','-lxilstandalone','-lxiltimer',
             '-lxilskey','-lxil','-lgcc','-lc','-Wl,--end-group'])
    (out/'commands.json').write_text(json.dumps(commands,indent=2),encoding='utf-8')
    print(json.dumps({'result':'PASS','elf':str(elf),'sha256':hashlib.sha256(elf.read_bytes()).hexdigest(),
                      'note':'Rebuilt binary has a new BUILD timestamp; frozen delivery ELF is unchanged.'},indent=2))

if __name__=='__main__':main()

