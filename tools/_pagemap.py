"""Map every PDF page of the TM-73254 copy: text length, embedded images, and any TABLE hits."""
from pypdf import PdfReader

r = PdfReader(r'reference\NASA-TM-73254_UH-1H-force-and-moment-model.pdf')
out = []
out.append('total pages = %d' % len(r.pages))
for i, page in enumerate(r.pages, start=1):
    try:
        txt = page.extract_text() or ''
    except Exception as exc:  # noqa: BLE001
        txt = ''
        out.append('page %d: extract_text failed %r' % (i, exc))
    flat = ' '.join(txt.split())
    try:
        nimg = len(list(page.images))
    except Exception:  # noqa: BLE001
        nimg = -1
    out.append('--- page %d : chars=%d images=%d' % (i, len(flat), nimg))
    out.append('    ' + flat[:400])
    low = flat.upper()
    for tag in ('TABLE 1', 'TABLE 2', 'TABLE 3', 'TABLE 4', 'TABLE 5',
                'SUMMARY OF UH-1H', 'CONTROL TRAVELS', 'LINKAGE'):
        pos = low.find(tag)
        if pos >= 0:
            out.append('    HIT[%s] @%d : %s' % (tag, pos, flat[max(0, pos - 80):pos + 200]))

open('_pagemap.txt', 'w', encoding='utf-8').write('\n'.join(out))
print('done', len(r.pages))