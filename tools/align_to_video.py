import sys, wave, numpy as np
W=sys.argv[1]
def load(p):
    w=wave.open(p); n=w.getnframes(); d=np.frombuffer(w.readframes(n),dtype=np.int16).astype(np.float32)/32768; return d
v=load(W+'/video16k.wav'); r=load(W+'/render16k.wav'); sr=16000; hop=80   # 5 ms
def env(x):
    n=len(x)//hop; e=np.sqrt((x[:n*hop].reshape(n,hop)**2).mean(1)+1e-10); le=np.log(e); d=np.diff(le,prepend=le[0]); d[d<0]=0; return d
ev=env(v); er=env(r)
# global offset: video_time = render_time + off ; search 0..20 s
N=1<<int(np.ceil(np.log2(len(ev)+len(er))))
c=np.fft.irfft(np.fft.rfft(ev,N)*np.conj(np.fft.rfft(er,N)),N)
lags=np.arange(N); lags[lags>N//2]-=N
ok=(lags>=0)&(lags<=4000)
best=lags[ok][np.argmax(c[ok])]; off=best*hop/sr
srt=np.sort(c[ok])[::-1]
print('global offset %.3f s (expected ~8.75 from clocks), peak ratio %.2f'%(off, srt[0]/srt[50]))
# per-batch refinement against video onsets
lines=[l.split() for l in open(W+'/take_midi.txt')]
t0=float(lines[0][0]); ts=np.array([float(l[0])-t0 for l in lines])
isnote=np.array([(int(l[1],16)&0xF0)==0x90 and int(l[3])>0 for l in lines])
new=ts.copy(); shifts=[]
for bt in np.unique(ts):
    idx=np.where(ts==bt)[0]
    if not isnote[idx].any(): new[idx]=bt-0.015; continue
    lo=int((bt+off-0.050)*sr/hop); hi=int((bt+off+0.010)*sr/hop)
    lo=max(lo,0); hi=min(hi,len(ev)-1)
    if hi<=lo: new[idx]=bt-0.015; continue
    seg=ev[lo:hi]; k=np.argmax(seg)
    if seg[k] > 0.15:   # a clear onset
        vt=(lo+k)*hop/sr; s=vt-off-bt; new[idx]=bt+s; shifts.append(s)
    else: new[idx]=bt-0.015
shifts=np.array(shifts)
print('note batches snapped: %d, shift ms median %.1f, p10 %.1f, p90 %.1f'%(len(shifts),1000*np.median(shifts),1000*np.percentile(shifts,10),1000*np.percentile(shifts,90)))
np.savetxt(W+'/times.txt', np.maximum(new,0), fmt='%.6f')
open(W+'/offset.txt','w').write('%.4f\n'%off)
