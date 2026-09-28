"""Wrap the CCITT page scans of the mangled table pages as TIFF for Windows WIC."""
import io
import struct

from pypdf import PdfReader

r = PdfReader(r'reference\NASA-TM-73254_UH-1H-force-and-moment-model.pdf')
log = []
for i in range(23, 33):
    page = r.pages[i - 1]
    xo = page.get('/Resources', {}).get('/XObject')
    if not xo:
        log.append('page %d: no xobjects' % i)
        continue
    for name in xo:
        d = xo[name].get_object()
        if d.get('/Subtype') != '/Image':
            continue
        parms = d.get('/DecodeParms', {})
        log.append('page %d %s filter=%s parms=%s len=%s' % (
            i, name, d.get('/Filter'), dict(parms) if parms else None, d.get('/Length')))
        try:
            data = d.get_data()
        except Exception as exc:  # noqa: BLE001
            log.append('   get_data failed: %r' % (exc,))
            data = getattr(d, '_data', b'')
        log.append('   bytes=%d' % len(data))
        w = int(d['/Width'])
        h = int(d['/Height'])
        black_is_1 = bool(parms.get('/BlackIs1', False)) if parms else False
        k = int(parms.get('/K', -1)) if parms else -1
        comp = 4 if k < 0 else 3
        entries = [
            (256, 4, w),                    # ImageWidth
            (257, 4, h),                    # ImageLength
            (258, 3, 1),                    # BitsPerSample
            (259, 3, comp),                 # Compression
            (262, 3, 1 if black_is_1 else 0),  # PhotometricInterpretation
            (266, 3, 1),                    # FillOrder
            (277, 3, 1),                    # SamplesPerPixel
            (278, 4, h),                    # RowsPerStrip
            (279, 4, len(data)),            # StripByteCounts
            (292, 4, 1),                    # T4Options: 2D (G4)
            (273, 4, 0),                    # StripOffsets (patched below)
        ]
        entries.sort()
        ifd_size = 2 + 12 * len(entries) + 4
        data_off = 8 + ifd_size
        out = bytearray()
        out += b'II' + struct.pack('<HI', 42, 8)
        out += struct.pack('<H', len(entries))
        for tag, typ, val in entries:
            if tag == 273:
                val = data_off
            if typ == 3:
                out += struct.pack('<HHHH', tag, typ, 1, val & 0xFFFF)
            else:
                out += struct.pack('<HHII', tag, typ, 1, val)
        out += struct.pack('<I', 0)
        out += data
        path = '_imgs/p%02d.tif' % i
        with io.open(path, 'wb') as fh:
            fh.write(out)
        log.append('   wrote %s (%d bytes)' % (path, len(out)))
io.open('_tiff_out.txt', 'w', encoding='utf-8').write('\n'.join(log))
print('done')