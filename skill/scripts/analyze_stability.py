"""Raw REFCHK OADEV audit. No third-party dependency; never use KF X[1] as physical output."""
from pathlib import Path
import argparse,json,math,re,statistics

def kv(line):
    return {k:float(v) for k,v in re.findall(r'(\w+)=([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)',line)}

def records(path):
    out=[];row=None
    for line in path.read_text(encoding='utf-8-sig',errors='replace').splitlines():
        if line.startswith('REFCHK '):
            if row is not None:out.append(row)
            row=kv(line)
        elif row is not None:
            if line.startswith('[TUNING]'):row['state']='TUNING';row['control']=kv(line)
            elif line.startswith('[CALIB]'):row['state']='CALIB';row['control']=kv(line)
            elif line.startswith('[HOLDOVER]'):row['state']='HOLDOVER';row['control']=kv(line)
            elif line.startswith('SAMP '):row['samp']=kv(line)
            elif line.startswith('CFUSE '):row['cfuse']=kv(line)
    if row is not None:out.append(row)
    return [r for r in out if 'state' in r and all(k in r for k in ['ms','seq0','seq1','raw'])]

def oadev(y,m):
    if len(y)<2*m:return None
    c=[0.]
    for v in y:c.append(c[-1]+v)
    av=[(c[i+m]-c[i])/m for i in range(len(y)-m+1)]
    d=[av[i+m]-av[i] for i in range(len(av)-m)]
    return math.sqrt(statistics.fmean(v*v for v in d)/2)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('log',type=Path)
    ap.add_argument('--start-ms',type=int,default=1200000);ap.add_argument('--end-ms',type=int,default=3600000)
    ap.add_argument('--output',type=Path);a=ap.parse_args()
    rows=[r for r in records(a.log) if a.start_ms<=r['ms']<a.end_ms]
    if len(rows)<2:raise ValueError('Too few complete records')
    if rows[0]['ms']!=a.start_ms or rows[-1]['ms']!=a.end_ms-1000:raise ValueError('Requested interval is incomplete')
    for r in rows:
        if r['seq0']!=r['seq1']:raise ValueError('Torn snapshot')
        if 'samp' not in r or r['samp'].get('why')!=0 or r['samp'].get('acc')!=1:raise ValueError('Rejected/model-filled/unknown sample')
    for x,y in zip(rows,rows[1:]):
        if y['ms']-x['ms']!=1000 or (int(y['seq0'])-int(x['seq0']))%2**32!=1:raise ValueError('Time/sequence gap; split the record')
    if len({r['state'] for r in rows})!=1:raise ValueError('State transition in analysis interval')
    freq=[r['raw']*.1953125 for r in rows]
    cf=[r['cfuse'] for r in rows if 'cfuse' in r]
    result={'input':str(a.log),'n':len(rows),'ms':[rows[0]['ms'],rows[-1]['ms']],
            'mean_ppb':statistics.fmean(freq),'mean_mhz_20mhz':statistics.fmean(freq)*20,
            'mean_time_drift_ns_per_s':statistics.fmean(freq),'phase_increment_sum_ns':sum(freq),
            'OADEV_ppb':{str(m):oadev(freq,m) for m in [1,5,10,50,100,200,300,400,600]},
            'cfuse_updates':len(cf),'cfuse_positive':sum(r.get('used_u',0)>0 for r in cf),
            'cfuse_mean_weight':statistics.fmean(r.get('used_u',0)/1e6 for r in cf) if cf else None,
            'warning':'OADEV is relative to this GNSS reference. Equivalent ns/s is a fluctuation scale, not directional drift.'}
    if a.output:
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
