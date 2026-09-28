"""Dump any embedded images on the mangled table pages of the TM-73254 copy."""
import io
import os

from pypdf import PdfReader

r = PdfReader(r'reference\NASA-TM-73254_UH-1H-force-and-moment-model.pdf')
log = []
os.makedirs('_imgs', exist_ok=True)
for i in range(23, 30):
    page = r.pages[i - 1]
    imgs = list(page.images)
    log.append('page %d: %d image(s)' % (i, len(imgs)))
    for j, im in enumerate(imgs):
        data = im.data
        name = '%s' % (im.name or 'img%d' % j)
        ext = os.path.splitext(name)[1] or '.bin'
        path = '_imgs/p%02d_%d%s' % (i, j, ext)
        with open(path, 'wb') as fh:
            fh.write(data)
        log.append('   %s -> %s (%d bytes)' % (name, path, len(data)))
io.open('_img_out.txt', 'w', encoding='utf-8').write('\n'.join(log))
print('\n'.join(log))