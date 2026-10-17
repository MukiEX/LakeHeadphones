import numpy as np, sys
a=np.fromfile(sys.argv[1],np.int16).astype(np.int32); b=np.fromfile(sys.argv[2],np.int16).astype(np.int32)
n=min(len(a),len(b)); d=np.abs(a[:n]-b[:n])
sig=np.sqrt(np.mean(a[:n].astype(float)**2)); err=np.sqrt(np.mean(d.astype(float)**2))
print(f"samples {n}  max|diff| {d.max()}  mean {d.mean():.4f}  >1LSB {np.sum(d>1)}  SNR {20*np.log10(sig/max(err,1e-9)):.1f} dB")
sys.exit(0 if d.max()<=2 else 1)
