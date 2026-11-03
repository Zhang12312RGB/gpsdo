"""Export recent raw GPSDO logs without inventing missing fields or timestamps.

Python 3.11 standard library. All raw files remain unchanged.
"""
from pathlib import Path
from collections import Counter
import argparse,csv,hashlib,json,re

SCHEMA='gpsdo_raw_csv_1'
RX=re.compile(r'(\w+)=([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)')
FIELDS=['record_index','boot_ms','boot_s','seq0','seq1','state','record_complete',
 'refchk_line','state_line','raw_q8','raw_phase_increment_ns','raw_frequency_ppb_1s',
 'ocxo_delta_count','fclk_delta_count','fraction_q16','status','dac_snapshot','dac_command',
 'X0_ns','X1_ppb','Tu_s','R_ns2','mode','phase_control_enabled',
 'fix','sat','hdop_display','hdop','cn0_display','cn0_dbhz',
 'sample_reason','acc_used','kf_used','gapfill_count','hard_reset_count',
 'nmea_fresh','gga_age_ms','cfuse_used_weight','cfuse_reason',
 'slow_src','slow_candidate_valid','slow_span_s','slow_age_s','slow_lag_s',
 'continuous_good_segment']
PREFIXES={'SLOWDBG ':'slowdbg','SLOWC ':'slowc','CFUSE ':'cfuse',
          'SAMP ':'samp','NMEA chk:':'nmea','STATE ->':'state_events'}

def kv(line):
    out={}
    for k,v in RX.findall(line):
        try:out[k]=float(v) if any(c in v for c in '.eE') else int(v)
        except ValueError:continue
    return out

def display_value(line,key,places):
    m=re.search(r'\b'+key+r'=([-+]?\d+)\.(\d+)',line)
    if not m:return '', ''
    whole,frac=m.groups();sign=-1 if whole.startswith('-') else 1
    # Old logging separately rounded fractional digits, e.g. cn0=35.10.
    return m.group(0).split('=',1)[1],int(whole)+sign*int(frac)/10**places

def parse(path):
    rows=[];row=None;latest_nmea={};events={v:[] for v in PREFIXES.values()}
    orphan=[];configs=[]
    for lineno,line in enumerate(path.read_text(encoding='utf-8-sig',errors='replace').splitlines(),1):
        if line.startswith(('===== BUILD','KFCONF ','CFCONF ','FINALCONF ')):configs.append(line)
        if line.startswith('REFCHK '):
            if row is not None:rows.append(row)
            row={'ref':kv(line),'refchk_line':lineno,'nmea':dict(latest_nmea)}
        state=re.match(r'^\[(TUNING|CALIB|HOLDOVER)\]',line)
        if state:
            if row is None:orphan.append({'line':lineno,'text':line})
            else:
                row.update(state=state.group(1),control=kv(line),state_line=lineno)
                row['hdop_display'],row['hdop']=display_value(line,'hdop',2)
                row['cn0_display'],row['cn0']=display_value(line,'cn0',1)
        for prefix,name in PREFIXES.items():
            if line.startswith(prefix):
                values=kv(line)
                ev={'line_number':lineno,'context_previous_ref_ms':row['ref'].get('ms','') if row else '',
                    'raw_line':line,**values}
                events[name].append(ev)
                if name=='nmea':latest_nmea=values
                elif row is not None:row[name]=values
    if row is not None:rows.append(row)
    return rows,events,orphan,configs

def normalized(rows):
    output=[];prev=None;segment=0;prev_good=False
    for i,r in enumerate(rows,1):
        f=r['ref'];c=r.get('control',{});s=r.get('samp',{});n=r['nmea'];u=r.get('cfuse',{});slow=r.get('slowc',{})
        val=lambda v,k,scale=1:v[k]*scale if k in v else ''
        complete='state' in r and all(k in f for k in ['ms','seq0','seq1','raw'])
        good=complete and f['seq0']==f['seq1'] and s.get('why')==0 and s.get('acc')==1 and f['raw']!=2147483647
        continuous=prev is not None and f.get('ms',0)-prev['ref'].get('ms',0)==1000 and ((int(f.get('seq0',0))-int(prev['ref'].get('seq0',0)))%2**32==1) and r.get('state')==prev.get('state')
        if good and not (prev_good and continuous):segment+=1
        o=dict.fromkeys(FIELDS,'')
        o.update(record_index=i,boot_ms=f.get('ms',''),boot_s=val(f,'ms',.001),seq0=f.get('seq0',''),seq1=f.get('seq1',''),
          state=r.get('state',''),record_complete=int(complete),refchk_line=r['refchk_line'],state_line=r.get('state_line',''),
          raw_q8=f.get('raw',''),raw_phase_increment_ns=val(f,'raw',.1953125),raw_frequency_ppb_1s=val(f,'raw',.1953125),
          ocxo_delta_count=f.get('ocxo',''),fclk_delta_count=f.get('clk',''),fraction_q16=f.get('frac',''),status=f.get('status',''),
          dac_snapshot=f.get('D',''),dac_command=c.get('dac',''),X0_ns=val(c,'phsm',.001),X1_ppb=val(c,'frqm',.001),
          Tu_s=c.get('T',''),R_ns2=c.get('R',''),mode=c.get('fmod',''),phase_control_enabled=c.get('phsg',''),
          fix=c.get('fix',''),sat=c.get('sat',''),hdop_display=r.get('hdop_display',''),hdop=r.get('hdop',''),
          cn0_display=r.get('cn0_display',''),cn0_dbhz=r.get('cn0',''),
          sample_reason=s.get('why',''),acc_used=s.get('acc',''),kf_used=s.get('used',''),gapfill_count=s.get('gapfill',''),
          hard_reset_count=s.get('hreset',''),nmea_fresh=n.get('fresh',''),gga_age_ms=n.get('gga_age',''),
          cfuse_used_weight=val(u,'used_u',1e-6),cfuse_reason=u.get('reason',''),slow_src=slow.get('src',''),
          slow_candidate_valid=slow.get('v',''),slow_span_s=slow.get('span',''),slow_age_s=slow.get('age',''),slow_lag_s=slow.get('lag',''),
          continuous_good_segment=segment if good else '')
        output.append(o);prev=r;prev_good=good
    return output

def write_csv(path,rows,fields=None):
    if fields is None:fields=list(dict.fromkeys(k for r in rows for k in r)) or ['line_number','context_previous_ref_ms','raw_line']
    with path.open('w',encoding='utf-8-sig',newline='') as stream:
        w=csv.DictWriter(stream,fieldnames=fields);w.writeheader();w.writerows(rows)

def export_one(path,root,out):
    rows,events,orphans,configs=parse(path);main=normalized(rows)
    write_csv(out/(path.stem+'.csv'),main,FIELDS)
    for name,items in events.items():write_csv(out/(path.stem+'_'+name+'.csv'),items)
    summary={'raw_path':path.relative_to(root).as_posix(),'raw_bytes':path.stat().st_size,
      'raw_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
      'main_csv':(out/(path.stem+'.csv')).relative_to(root).as_posix() if out.is_relative_to(root) else str(out/(path.stem+'.csv')),
      'refchk_records':len(rows),'complete_records':sum(r['record_complete'] for r in main),
      'partial_records':sum(not r['record_complete'] for r in main),
      'state_counts':dict(Counter(r['state'] or 'UNKNOWN' for r in main)),
      'time_gap_transitions':sum(b.get('boot_ms',0)-a.get('boot_ms',0)!=1000 for a,b in zip(main,main[1:]) if a['boot_ms']!='' and b['boot_ms']!=''),
      'event_counts':{k:len(v) for k,v in events.items()},'orphan_state_lines':orphans,'configuration_lines':configs}
    return summary

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('log',nargs='?',type=Path)
    ap.add_argument('--all-recent',action='store_true',help='Export archived v14 through v26')
    ap.add_argument('--output',type=Path)
    a=ap.parse_args();root=Path(__file__).resolve().parents[2]
    out=(a.output or root/'data/csv/current').resolve();out.mkdir(parents=True,exist_ok=True)
    paths=sorted(p for p in (root/'data/raw_support').glob('*.dat') if re.search(r'_v(?:1[4-9]|2[0-6])\.dat$',p.name)) if a.all_recent else [a.log]
    if not paths or any(p is None or not p.is_file() for p in paths):ap.error('Supply a raw log or --all-recent')
    summaries=[export_one(p.resolve(),root,out) for p in paths]
    write_csv(out/'csv_index.csv',[{'raw_path':r['raw_path'],'main_csv':r['main_csv'],'raw_sha256':r['raw_sha256'],
      'refchk_records':r['refchk_records'],'complete_records':r['complete_records'],'partial_records':r['partial_records'],
      'time_gap_transitions':r['time_gap_transitions']} for r in summaries])
    files=[{'path':p.relative_to(root).as_posix() if p.is_relative_to(root) else str(p),
            'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(out.glob('*.csv'))]
    result={'schema':SCHEMA,'raw_files_modified':False,'datasets':summaries,'csv_files':files}
    (out/'csv_manifest.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'result':'PASS','datasets':len(summaries),'main_rows':sum(r['refchk_records'] for r in summaries),
      'partial_records':sum(r['partial_records'] for r in summaries),'output':str(out)},ensure_ascii=False))

if __name__=='__main__':main()
