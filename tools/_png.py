"""Export the scanned page images of the table pages as PNG for viewing."""
import io
import os

from pypdf import PdfReader

r = PdfReader(r'reference\NASA-TM-73254_UH-1H-force-and-moment-model.pdf')
os.makedirs('_imgs', exist_ok=True)
log = []
for i in range(23, 29):
    page = r.pages[i - 1]
    for j, im in enumerate(page.images):
        img = im.image
        path = '_imgs/page%02d_%d.png' % (i, j)
        img.save(path)
        log.append('%s %s %s' % (path, im.name, img.size))
io.open('_png_out.txt', 'w', encoding='utf-8').write('\n'.join(log))
print('done')