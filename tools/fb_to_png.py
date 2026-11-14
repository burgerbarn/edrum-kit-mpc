import sys,zlib,struct
raw=open(sys.argv[1],'rb').read(); hdr=raw[:32].replace(b'\x00',b' ').split(); W,H,P,B=map(int,hdr); px=raw[32:32+P*H]
rot=sys.argv[3] if len(sys.argv)>3 else 'cw'
print('nonzero sample', sum(1 for b in px[:2000000:997] if b))
OW,OH=(H,W)
rows=[]
for y in range(OH):
    r=bytearray(b'\x00')
    for x in range(OW):
        if rot=='cw': sx,sy=y,H-1-x
        else: sx,sy=W-1-y,x
        o=sy*P+sx*4; r+=bytes((px[o+2],px[o+1],px[o]))
    rows.append(bytes(r))
def ch(t,d): return struct.pack('>I',len(d))+t+d+struct.pack('>I',zlib.crc32(t+d)&0xffffffff)
open(sys.argv[2],'wb').write(b'\x89PNG\r\n\x1a\n'+ch(b'IHDR',struct.pack('>IIBBBBB',OW,OH,8,2,0,0,0))+ch(b'IDAT',zlib.compress(b''.join(rows),6))+ch(b'IEND',b''))
