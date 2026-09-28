"""Map PDF pages of the TM-73254 copy to report tables."""
import io

from pypdf import PdfReader

r = PdfReader(r'reference\NASA-TM-73254_UH-1H-force-and-moment-model.pdf')
print('pages:', len(r.pages))
out = []
for i, p in enumerate(r.pages, 1):
    if not (18 <= i <= 34):
        continue
    box = p.mediabox
    t = (p.extract_text() or '')
    flat = ' '.join(t.split())
    out.append('%2d size=%.0fx%.0f rot=%s | %s' % (
        i, float(box.width), float(box.height), p.get('/Rotate'), flat[:260]))
io.open('_p2_out.txt', 'w', encoding='utf-8').write('\n'.join(out))
print('done')