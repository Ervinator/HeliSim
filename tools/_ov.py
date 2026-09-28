from PIL import Image

for p in range(24, 29):
    src = '_imgs/page%d_0.png' % p
    im = Image.open(src)
    w, h = im.size
    tw = 1500
    ov = im.resize((tw, int(h * tw / w)), Image.LANCZOS)
    dst = '_imgs/ov%d.png' % p
    ov.save(dst)
    print(dst, w, h, '->', ov.size)