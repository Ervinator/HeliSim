"""Dump the full extractable text of the TM-73254 Table 1/2/3 pages (PDF pages 21-28)."""
from pypdf import PdfReader

r = PdfReader(r'reference\NASA-TM-73254_UH-1H-force-and-moment-model.pdf')
for i in range(21, 29):
    txt = r.pages[i - 1].extract_text() or ''
    with open('_pg%02d.txt' % i, 'w', encoding='utf-8') as fh:
        fh.write(txt)
    print('page %d -> %d chars' % (i, len(txt)))