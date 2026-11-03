"""Recompute cited raw OADEV and the 331 frozen R windows from portable CSVs.

Outputs are generated separately from the archived measurement and result files.
Stability needs only Python; --r needs NumPy; --plots also needs Matplotlib.
"""
from pathlib import Path
import argparse,csv,hashlib,importlib.util,json,math,statistics

def read_csv(p):
    with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))

def load_module(name,p):
    s=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--r',action='store_true',help='Check local residual labels and whole-run regression')
    ap.add_argument('--plots',action='store_true',help='Draw raw OADEV and signed X[1] charts')
    ap.add_argument('--output',type=Path)
    a=ap.parse_args();root=Path(__file__).resolve().parents[2]
    out=(a.output or root/'data/reproduced').resolve();out.mkdir(parents=True,exist_ok=True)
    audit=load_module('audit',Path(__file__).with_name('analyze_stability.py'))
    exports=load_module('raw_csv',Path(__file__).with_name('export_raw_csv.py'))
    csvroot=root/'data/csv/current'
    manifest=json.loads((csvroot/'csv_manifest.json').read_text(encoding='utf-8'))
    for d in manifest['datasets']:
        raw=root/d['raw_path']
        assert hashlib.sha256(raw.read_bytes()).hexdigest()==d['raw_sha256'],raw
    for f in manifest['csv_files']:
        path=root/f['path'];assert hashlib.sha256(path.read_bytes()).hexdigest()==f['sha256'],path
    selected={}
    reference=json.loads((root/'data/results/slow_loop_three_runs.json').read_text(encoding='utf-8'))
    for version in [23,24,25]:
        name=f'log_20261031_v{version}'
        r=read_csv(csvroot/(name+'.csv'))
        r=[p for p in r if p['boot_ms'] and 1200000<=int(p['boot_ms'])<3600000]
        assert len(r)==2400 and all(p['record_complete']=='1' and p['state']=='TUNING' and p['sample_reason']=='0' and p['acc_used']=='1' for p in r)
        for x,y in zip(r,r[1:]):assert int(y['boot_ms'])-int(x['boot_ms'])==1000 and (int(y['seq0'])-int(x['seq0']))%2**32==1
        y=[float(p['raw_frequency_ppb_1s']) for p in r]
        values={str(m):audit.oadev(y,m) for m in [1,5,10,50,100,200,300,400,600]}
        old=next(p for p in reference if p['version']==version)
        assert all(abs(values[k]-v)<1e-10 for k,v in old['OADEV'].items())
        used=[p for p in r if p['cfuse_used_weight']!='']
        selected[name]={'n':len(r),'OADEV_ppb':values,'mean_raw_ppb':statistics.fmean(y),
          'raw_phase_increment_sum_ns':sum(y),'fusion_updates':len(used),
          'fusion_positive':sum(float(p['cfuse_used_weight'])>0 for p in used),
          'fusion_mean_weight':statistics.fmean(float(p['cfuse_used_weight']) for p in used)}
    result={'result':'PASS','analysis_ms':[1200000,3600000],'end_exclusive':True,'samples_per_run':2400,
            'stability':selected,'raw_and_csv_sha256_checked':True,'hardware_newly_tested':False}

    if a.r:
        import numpy as np
        frozen=read_csv(root/'data/results/r_parameter_reasonableness_labels_20261101.csv')
        frozen=[r for r in frozen if r['run'] in ['v14','v22','v26'] and r['state']=='CALIB']
        assert len(frozen)==331
        data={}
        for run in ['v14','v22','v26']:
            date='20261030' if run=='v14' else ('20261031' if run=='v22' else '20261101')
            byms={}
            for p in read_csv(csvroot/f'log_{date}_{run}.csv'):
                if p['boot_ms']:byms.setdefault(int(p['boot_ms']),[]).append(p)
            data[run]=byms
        labels=[];largest=0.;feature_diff=0.
        for f in frozen:
            ms=int(float(f['start_ms']));n=int(f['W']);points=[]
            for k in range(n):
                candidates=[p for p in data[f['run']].get(ms+1000*k,[]) if p['state']=='CALIB' and p['dac_command'] and int(p['dac_command'])==int(float(f['D']))]
                assert len(candidates)==1,(f['run'],ms,k)
                p=candidates[0];assert p['sample_reason']=='0' and p['acc_used']=='1' and p['record_complete']=='1'
                points.append(p)
            t=np.arange(n,dtype=float);tc=t-t.mean();ss=float(tc@tc)
            ph=np.cumsum([float(p['raw_phase_increment_ns']) for p in points]);ph-=ph[0]
            slope=float(tc@ph/ss);res=ph-ph.mean()-slope*tc
            R=float(res@res/(n-2));largest=max(largest,abs(R-float(f['R_ns2'])))
            weights=(1-1/n-tc**2/ss)/(n-2)
            h=float(weights@np.array([float(p['hdop']) for p in points]));sat=float(weights@np.array([float(p['sat']) for p in points]))
            inv=float(weights@np.array([1/float(p['cn0_dbhz']) for p in points]))
            feature_diff=max(feature_diff,abs(h-float(f['hdop'])),abs(sat-float(f['sat'])),abs(inv-float(f['inv_cn0'])))
            labels.append({'run':f['run'],'start_ms':ms,'W':n,'R_ns2':R,'hdop':h,'sat':sat,'inv_cn0':inv})
        assert largest<1e-5,largest
        assert feature_diff<1e-10,feature_diff
        folds=[]
        for run in ['v14','v22','v26']:
            train=[p for p in labels if p['run']!=run];test=[p for p in labels if p['run']==run]
            X=np.array([[p[k] for k in ['hdop','sat','inv_cn0']]+[1.] for p in train]);y=np.array([p['R_ns2'] for p in train])
            A=np.array([[p[k] for k in ['hdop','sat','inv_cn0']]+[1.] for p in test]);z=np.array([p['R_ns2'] for p in test])
            beta=np.linalg.lstsq(X,y,rcond=None)[0]
            folds.append({'held_out':run,'n':len(test),'constant_RMSE_ns2':float(np.sqrt(np.mean((y.mean()-z)**2))),
                'linear4_RMSE_ns2':float(np.sqrt(np.mean((A@beta-z)**2)))})
        agg={k:math.sqrt(statistics.fmean(p[k]**2 for p in folds)) for k in ['constant_RMSE_ns2','linear4_RMSE_ns2']}
        assert abs(agg['constant_RMSE_ns2']-11.7157913071)<1e-7
        assert abs(agg['linear4_RMSE_ns2']-12.2018820480)<1e-7
        with (out/'r_labels_331_recomputed.csv').open('w',encoding='utf-8-sig',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(labels[0]));writer.writeheader();writer.writerows(labels)
        result['R']={'windows':331,'window_counts':dict((r,sum(p['run']==r for p in labels)) for r in ['v14','v22','v26']),
            'maximum_label_difference_ns2':largest,'maximum_feature_difference':feature_diff,
            'whole_run_holdout':folds,'equal_run_aggregate':agg,'runtime_R_ns2':80,
            'label_is_pure_measurement_variance':False}
    if a.plots:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,ax=plt.subplots(figsize=(8.5,5))
        for name,label in [('log_20261031_v23','Slow OFF v23'),('log_20261031_v24','Limited C ON v24'),('log_20261031_v25','Slow OFF v25')]:
            v=selected[name]['OADEV_ppb'];xs=sorted(int(k) for k in v)
            ax.loglog(xs,[v[str(m)] for m in xs],'-o',label=label,markersize=4)
        ax.set(xlabel='Averaging time tau / s',ylabel='Raw-channel OADEV / ppb',title='Same 2400-s window; relative to GNSS reference')
        ax.grid(which='both',alpha=.2);ax.legend();fig.tight_layout();fig.savefig(out/'raw_oadev_three_runs.png',dpi=200);plt.close(fig)
        rows=read_csv(csvroot/'log_20261031_v24.csv');rows=[p for p in rows if p['record_complete']=='1' and p['X1_ppb']!='']
        fig,axs=plt.subplots(2,1,figsize=(9,6))
        axs[0].step([int(p['boot_ms'])/1000 for p in rows],[float(p['X1_ppb']) for p in rows],where='post')
        rows=[p for p in rows if 1200000<=int(p['boot_ms'])<3600000]
        axs[1].step([int(p['boot_ms'])/1000 for p in rows],[float(p['X1_ppb']) for p in rows],where='post')
        for ax in axs:ax.set(xlabel='Time since boot / s',ylabel='KF X[1] / ppb');ax.grid(alpha=.2)
        axs[0].set_title('Historical v24, same normal control configuration as the frozen final ELF')
        axs[1].set_title('Steady-state zoom: 1200 to 3599 s')
        fig.tight_layout();fig.savefig(out/'x1_time_v24.png',dpi=200);plt.close(fig)
    (out/'reproduction_results.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'result':'PASS','stability_runs':3,'R_checked':a.r,'plots':a.plots,'output':str(out)},ensure_ascii=False))

if __name__=='__main__':main()
