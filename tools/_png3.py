"""Export scanned page images of PDF pages 29-32 as PNG plus small overviews."""
import io
import os

from PIL import Image
from pypdf import PdfReader

r = PdfReader(r'reference\NASA-TM-73254_UH-1H-force-and-moment-model.pdf')
os.makedirs('_imgs', exist_ok=True)
log = []
for i in range(29, 33):
    page = r.pages[i - 1]
    for j, im in enumerate(page.images):
        img = im.image
        path = '_imgs/page%02d_%d.png' % (i, j)
        img.save(path)
        log.append('%s %s %s' % (path, im.name, img.size))
        tw = 1500
        ov = img.convert('L').resize((tw, int(img.size[1] * tw / img.size[0])), Image.LANCZOS)
        ovp = '_imgs/ov%02d_%d.png' % (i, j)
        ov.save(ovp)
        log.append('  -> %s %s' % (ovp, ov.size))
io.open('_png3_out.txt', 'w', encoding='utf-8').write('\n'.join(log))
print('done')