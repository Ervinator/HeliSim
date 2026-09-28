"""Dump raw text-layer runs (x, y, text) for a page.

Usage: python _pdfraw.py <pdf> <page> [max_runs]
"""
import sys

from pypdf import PdfReader


def mmul(m, n):
    return (
        m[0] * n[0] + m[1] * n[2],
        m[0] * n[1] + m[1] * n[3],
        m[2] * n[0] + m[3] * n[2],
        m[2] * n[1] + m[3] * n[3],
        m[4] * n[0] + m[5] * n[2] + n[4],
        m[4] * n[1] + m[5] * n[3] + n[5],
    )


def main(argv):
    reader = PdfReader(argv[1])
    page = reader.pages[int(argv[2]) - 1]
    limit = int(argv[3]) if len(argv) > 3 else 10 ** 9
    print("mediabox:", page.mediabox, "rotate:", page.get("/Rotate", 0))
    runs = []

    def visitor(text, cm, tm, font_dict, font_size):
        if text is None:
            return
        m = mmul(cm, tm)
        runs.append((m[4], m[5], m[0], m[1], text))

    page.extract_text(visitor_text=visitor)
    runs.sort(key=lambda r: (round(r[0], 1), round(r[1], 1)))
    for x, y, a, b, text in runs[:limit]:
        print("x=%8.2f y=%8.2f a=%6.3f b=%6.3f %r" % (x, y, a, b, text))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))