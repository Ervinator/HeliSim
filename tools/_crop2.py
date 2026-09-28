from PIL import Image

def crop(src, dst, top_frac, bot_frac, scale=2, left_frac=0.0, right_frac=1.0):
    im = Image.open(src)
    w, h = im.size
    box = (int(w * left_frac), int(h * top_frac), int(w * right_frac), int(h * bot_frac))
    c = im.crop(box)
    c = c.resize((c.width * scale, c.height * scale), Image.LANCZOS)
    c.save(dst)
    print(dst, c.size)

# PDF page 25 == printed page 18 == TABLE 3 continued (control-system constants)
crop('_imgs/page25_0.png', '_imgs/p25_kb.png', 0.20, 0.32, 5, 0.18, 0.62)
crop('_imgs/page25_0.png', '_imgs/p25_all.png', 0.08, 1.00, 4, 0.00, 0.75)