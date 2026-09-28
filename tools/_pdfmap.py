"""Map a PDF's text layer page by page (first lines + char count).

Usage: python _pdfmap.py <pdf>
Text-layer only; no raster or OCR processing.
"""
import sys

from pypdf import PdfReader


def main(argv):
    reader = PdfReader(argv[1])
    for i, page in enumerate(reader.pages):
        try:
            text = page.extract_text() or ""
        except Exception as exc:  # pragma: no cover
            text = "<<failed: %s>>" % exc
        lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
        head = " | ".join(lines[:3])[:120]
        print("%2d rot=%s chars=%5d  %s" % (i + 1, page.get("/Rotate", 0), len(text), head))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))