"""Locate pages whose text layer contains a needle, and report page geometry.

Usage:
    python _pdfscan.py <pdf> <needle> [--dump PAGE] [--layout]
This reads only the PDF's embedded text layer (no raster/OCR processing).
"""
import sys

from pypdf import PdfReader


def page_text(page, layout=False):
    try:
        if layout:
            return page.extract_text(extraction_mode="layout") or ""
        return page.extract_text() or ""
    except Exception as exc:  # pragma: no cover - diagnostics only
        return "<<extract failed: %s>>" % exc


def main(argv):
    path = argv[1]
    needle = argv[2] if len(argv) > 2 else ""
    dump = None
    layout = "--layout" in argv
    if "--dump" in argv:
        dump = int(argv[argv.index("--dump") + 1])

    reader = PdfReader(path)
    print("pages:", len(reader.pages))

    if dump is not None:
        page = reader.pages[dump - 1]
        print("--- page %d rotate=%s layout=%s ---" % (dump, page.get("/Rotate", 0), layout))
        print(page_text(page, layout))
        return 0

    for i, page in enumerate(reader.pages):
        text = page_text(page)
        if needle and needle.lower() in text.lower():
            print(
                "MATCH page %d rotate=%s chars=%d" % (i + 1, page.get("/Rotate", 0), len(text))
            )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))