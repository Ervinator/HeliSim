from PIL import Image

def crop(src, dst, top_frac, bot_frac, scale=2, left_frac=0.0, right_frac=1.0):
    im = Image.open(src)
    w, h = im.size
    box = (int(w * left_frac), int(h * top_frac), int(w * right_frac), int(h * bot_frac))
    c = im.crop(box)
    c = c.resize((c.width * scale, c.height * scale), Image.LANCZOS)
    c.save(dst)
    print(dst, c.size)

# PDF page 23 == printed page 16 == TABLE 2 (Summary of UH-1H physical constants)
crop('_imgs/page23_0.png', '_imgs/t2_top.png', 0.00, 0.34, 3)
crop('_imgs/page23_0.png', '_imgs/t2_bottom.png', 0.55, 1.00, 3)