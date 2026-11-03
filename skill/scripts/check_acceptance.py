"""Recompute v27 functionality evidence from archived raw bytes (standard library)."""
from pathlib import Path
from collections import Counter
import hashlib,importlib.util,json,re

def main():
 root=Path(__file__).resolve().parents[2]
 spec=importlib.util.spec_from_file_location('raw_export',root/'skill/scripts/export_raw_csv.py')
 m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
 raw=root/'data/raw_support/log_20261102_v27.dat'
 stored=json.loads((root/'sim/results/hardware_acceptance_20261102.json').read_text(encoding='utf-8'))
 assert hashlib.sha256(raw.read_bytes()).hexdigest()==stored['source_sha256']
 rows,events,orphan,configs=m.parse(raw)
 assert len(rows)==1154 and not orphan and all('state' in r for r in rows)
 assert configs==stored['configs']
 states=[]
 for r in rows:
  if not states or states[-1]!=r['state']:states.append(r['state'])
 assert states==['TUNING','CALIB','HOLDOVER','CALIB','TUNING'],states
 locked=[r for r in rows if r['state'] in ['CALIB','HOLDOVER']]
 assert len(locked)==354 and {r['control']['dac'] for r in locked}=={1851}
 assert all(r['samp']['hreset']==0 for r in rows)
 positive=[r for r in rows if r.get('cfuse',{}).get('used_u',0)>0]
 assert len(positive)==37
 assert all(r['slowc']['src']==2 and r['slowc']['v']==1 and r['cfuse']['reason']==0 for r in positive)
 assert all(b['ref']['ms']-a['ref']['ms']==10000 for a,b in zip(positive,positive[1:]))
 weights=[r['cfuse']['used_u']/1e6 for r in positive]
 assert min(weights)==.003333 and max(weights)==.060418
 assert weights[1]-weights[0]==.003334 or abs((weights[1]-weights[0])-.003334)<1e-9
 elf=root/'board/firmware/gpsdo_final_c.elf'
 assert hashlib.sha256(elf.read_bytes()).hexdigest()==stored['elf']['sha256']
 assert b'Nov  2 2026' in elf.read_bytes() and b'15:29:18' in elf.read_bytes()
 step=Counter(b['ref']['ms']-a['ref']['ms'] for a,b in zip(rows,rows[1:]))
 assert step=={1000:1151,1008:1,992:1}
 assert sum(x>1500 for x in step)==0
 result={'result':'PASS','complete_records':len(rows),'states':states,'locked_DAC':1851,
  'positive_fusion_updates':37,'actual_weight_range': [min(weights),max(weights)],
  'positive_ms_range':[positive[0]['ref']['ms'],positive[-1]['ref']['ms']],
  'time_steps_ms':dict(step),'hardware_memory_hash_readback':False,
  'scope':'CALIB loss/recovery and C participation; not a stability measurement'}
 print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
