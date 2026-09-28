"""Export scanned page images of PDF pages 12-14 as PNG plus small overviews.

Printed page = PDF page - 7, so these are printed pages 5-7: the
"Pilot to Swashplate Control Equations" / "Stabilizer Bar" section, where the
linkage constants (C1, C6, ...) and the stabilizer bar constants (KB, tauB)
are defined.
"""
import os

from PIL import Image
from pypdf import PdfReader

r = PdfReader(r'reference\NASA-TM-73254_UH-1H-force-and-moment-model.pdf')
os.makedirs('_imgs', exist_ok=True)
log = []
for i in (12, 13, 14):
    page = r.pages[i - 1]
    for j, im in enumerate(page.images):
        try:
            img = im.image
        except Exception as exc:            # pragma: no cover - diagnostic only
            log.append('%d %d FAILED %s' % (i, j, exc))
            continue
        path = '_imgs/page%02d_%d.png' % (i, j)
        img.save(path)
        log.append('%s %s %s' % (path, im.name, img.size))
        tw = 1500
        ov = img.convert('L').resize(
            (tw, int(img.size[1] * tw / img.size[0])), Image.LANCZOS)
        ovp = '_imgs/ov%02d_%d.png' % (i, j)
        ov.save(ovp)
        log.append('  -> %s %s' % (ovp, ov.size))
with open('_png4_out.txt', 'w', encoding='utf-8') as fh:
    fh.write('\n'.join(log))
print('\n'.join(log))