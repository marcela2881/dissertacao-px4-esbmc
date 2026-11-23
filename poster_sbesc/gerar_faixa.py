from PIL import Image, ImageDraw
import random
random.seed(7)
Wp, Hp = 3390, 470
im = Image.new('RGB', (Wp, Hp)); d = ImageDraw.Draw(im)
for y in range(Hp):
    t = y / Hp
    d.line([(0, y), (Wp, y)], fill=(int(11 + 20 * t * 0.6), int(58 + 53 * t * 0.6), int(99 + 79 * t * 0.6)))
def ridge(base, amp, col, step):
    pts = [(0, Hp)]; x = 0
    while x <= Wp + step:
        pts.append((x, base - random.random() * amp)); x += step
    pts.append((Wp, Hp)); d.polygon(pts, fill=col)
ridge(330, 70, (22, 86, 130), 90)
def trees(base, h, col, gap):
    x = -20
    while x < Wp + 20:
        hh = h * (0.7 + random.random() * 0.6); w = hh * 0.45
        d.polygon([(x, base), (x + w / 2, base - hh), (x + w, base)], fill=col)
        d.polygon([(x + w * 0.12, base - hh * 0.35), (x + w / 2, base - hh), (x + w * 0.88, base - hh * 0.35)], fill=col)
        x += gap * (0.7 + random.random() * 0.6)
trees(400, 95, (17, 72, 112), 38); trees(440, 110, (12, 58, 99), 34); trees(480, 120, (8, 44, 78), 30)
cx, cy = Wp - 300, 290   # drone baixo, a direita, fora do texto
for sx in (-1, 1):
    for sy in (-1, 1):
        rx, ry = cx + sx * 95, cy + sy * 55
        d.line([(cx, cy), (rx, ry)], fill=(235, 242, 248), width=9)
        d.ellipse([rx - 62, ry - 14, rx + 62, ry + 14], outline=(235, 242, 248), width=5)
d.rounded_rectangle([cx - 48, cy - 30, cx + 48, cy + 30], radius=16, fill=(235, 242, 248))
d.ellipse([cx - 12, cy - 12, cx + 12, cy + 12], fill=(11, 58, 99))
im.save('build/faixa_titulo.png')
