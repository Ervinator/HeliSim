"""Diagnose the mangled table pages: XObjects and text-matrix geometry."""
import io

from pypdf import PdfReader
from pypdf.generic import ContentStream

r = PdfReader(r'reference\NASA-TM-73254_UH-1H-force-and-moment-model.pdf')
out = []
for i in (22, 24, 25, 26, 27, 28):
    page = r.pages[i - 1]
    out.append('=== pdf page %d ===' % i)
    xo = page.get('/Resources', {}).get('/XObject')
    if xo:
        for name in xo:
            d = xo[name].get_object()
            out.append('  XObject %s subtype=%s filter=%s w=%s h=%s cs=%s bpc=%s' % (
                name, d.get('/Subtype'), d.get('/Filter'), d.get('/Width'),
                d.get('/Height'), d.get('/ColorSpace'), d.get('/BitsPerComponent')))
    else:
        out.append('  no XObjects')
    try:
        cs = ContentStream(page.get_contents(), r)
        ops = list(cs.operations)
    except Exception as exc:  # noqa: BLE001
        out.append('  content stream error: %r' % (exc,))
        continue
    names = {}
    for _operands, op in ops:
        names[op] = names.get(op, 0) + 1
    out.append('  ops: %s' % sorted(names.items(), key=lambda kv: -kv[1])[:20])
    fonts = page.get('/Resources', {}).get('/Font')
    if fonts:
        for fn in fonts:
            f = fonts[fn].get_object()
            out.append('  Font %s base=%s enc=%s' % (
                fn, f.get('/BaseFont'), f.get('/Encoding')))
    shown = 0
    for operands, op in ops:
        if op in (b'Tj', b'TJ', b"'", b'"') and shown < 25:
            out.append('    %s %r' % (op, operands))
            shown += 1
io.open('_cs_out.txt', 'w', encoding='utf-8').write('\n'.join(out))
print('lines', len(out))