import sys, random, math
from PIL import Image, ImageDraw, ImageFilter
P=sys.argv[1]
DPI=110; Wi=90/2.54; Hi=6.85
W,H=int(Wi*DPI),int(Hi*DPI)
random.seed(7)
# sky gradient: white at top -> pale mint/blue near the horizon
sky=Image.new('RGB',(W,H),'white'); d=ImageDraw.Draw(sky)
top=(255,255,255); bot=(214,236,238)
for y in range(H):
    t=min(1,(y/H)**1.4); d.line([(0,y),(W,y)],fill=tuple(int(top[i]+(bot[i]-top[i])*t) for i in range(3)))
img=sky.convert('RGBA')
def layer(): return Image.new('RGBA',(W,H),(0,0,0,0))
def conifer(dr,x,base,h,w,col):
    n=5
    dr.rectangle([x-w*0.03,base-h*0.12,x+w*0.03,base+4],fill=col)
    for j in range(n):
        bot=base-h*0.10-h*0.80*j/n*0.9
        apex=bot-h*0.9/n*2.0
        hw=w/2*(1-0.72*j/n)
        dr.polygon([(x,apex),(x-hw,bot),(x+hw,bot)],fill=col)
def forest(color,hmin,hmax,base,step,blur,jit=0.6,wr=0.42):
    L=layer(); dr=ImageDraw.Draw(L); x=-40
    while x<W+40:
        h=random.uniform(hmin,hmax)*DPI; conifer(dr,x,base*DPI,h,h*wr*2,color); x+=step*DPI*random.uniform(1-jit,1+jit)
    dr.rectangle([0,int(base*DPI),W,H],fill=color)
    return L.filter(ImageFilter.GaussianBlur(blur)) if blur else L
img=Image.alpha_composite(img,forest((196,222,214,150),0.9,1.25,6.25,0.55,4))
img=Image.alpha_composite(img,forest((150,196,176,200),0.55,0.85,6.4,0.42,2))
img=Image.alpha_composite(img,forest((78,150,118,235),0.45,0.7,6.55,0.36,1))
img=Image.alpha_composite(img,forest((24,92,70,255),0.4,0.65,6.8,0.30,0))
# mist band
m=layer(); dm=ImageDraw.Draw(m); dm.rectangle([0,int(5.95*DPI),W,int(6.2*DPI)],fill=(255,255,255,70)); img=Image.alpha_composite(img,m.filter(ImageFilter.GaussianBlur(25)))
# drone (top view, slightly tilted) drawn at 4x then downsampled
def drone(size, ang):
    s=size*4; D=Image.new('RGBA',(s,s),(0,0,0,0)); g=ImageDraw.Draw(D); c=s//2
    arm=s*0.36; ink=(27,36,48,255); ac=(14,143,154,255)
    for k in range(4):
        a=math.radians(45+90*k); ex,ey=c+arm*math.cos(a),c+arm*math.sin(a)
        g.line([(c,c),(ex,ey)],fill=ink,width=int(s*0.035))
        r=s*0.17
        rot=Image.new('RGBA',(s,s),(0,0,0,0)); rg=ImageDraw.Draw(rot)
        rg.ellipse([ex-r,ey-r,ex+r,ey+r],fill=(60,80,100,70),outline=(40,55,70,150),width=int(s*0.006))
        rg.line([(ex-r*0.95,ey),(ex+r*0.95,ey)],fill=(30,40,55,200),width=int(s*0.012))
        rg.line([(ex,ey-r*0.5),(ex,ey+r*0.5)],fill=(30,40,55,120),width=int(s*0.008))
        D=Image.alpha_composite(D,rot); g=ImageDraw.Draw(D)
        g.ellipse([ex-s*0.03,ey-s*0.03,ex+s*0.03,ey+s*0.03],fill=ink)
    g.rounded_rectangle([c-s*0.11,c-s*0.14,c+s*0.11,c+s*0.14],radius=int(s*0.05),fill=ink)
    g.rounded_rectangle([c-s*0.07,c-s*0.11,c+s*0.07,c-s*0.02],radius=int(s*0.02),fill=ac)
    g.ellipse([c-s*0.045,c+s*0.06,c+s*0.045,c+s*0.15],fill=(10,14,20,255),outline=ac,width=int(s*0.008))
    g.ellipse([c-s*0.02,c+s*0.09,c+s*0.02,c+s*0.13],fill=(90,130,160,255))
    return D.rotate(ang,resample=Image.BICUBIC).resize((size,size),Image.LANCZOS)
dsz=int(2.6*DPI); dr_img=drone(dsz,-12)
cx,cy=int(31.8*DPI),int(5.95*DPI)
# soft ground shadow on the treetops
sh=layer(); ImageDraw.Draw(sh).ellipse([cx-dsz*0.28,int(6.5*DPI),cx+dsz*0.28,int(6.8*DPI)],fill=(0,30,20,70))
img=Image.alpha_composite(img,sh.filter(ImageFilter.GaussianBlur(10)))
img.alpha_composite(dr_img,(cx-dsz//2,cy-dsz//2))
img.convert('RGB').save(P+'/build/banner_header.jpg',quality=92)
print(W,H)
