"""Cluster per-glyph text-layer runs into rotated table lines.

Usage: python _pdfrot2.py <pdf> <first_page> <last_page> [x_tol] [y_desc]

The TM-73254 constant tables are printed rotated 90 deg on portrait pages; the
embedded text layer stores one glyph per run, so a table ROW appears as a run of
glyphs sharing (almost) the same page x with increasing y. This script groups
glyphs by x and concatenates them in reading order.

Reads only the PDF text layer plus character origins (no raster analysis).
"""
import math
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


def glyphs(page):
    runs = []

    def visitor(text, cm, tm, font_dict, font_size):
        if not text:
            return
        m = mmul(cm, tm)
        runs.append((m[4], m[5], m[0], m[1], text))

    page.extract_text(visitor_text=visitor)
    return runs


def rotation_of(runs):
    worst = 0.0
    for _, _, a, b, _ in runs:
        ang = math.degrees(math.atan2(b, a))
        if abs(ang) > abs(worst):
            worst = ang
    return worst


def cluster_lines(runs, tol):
    ordered = sorted(runs, key=lambda r: (r[0], r[1]))
    groups = []
    for x, y, a, b, text in ordered:
        if groups and abs(x - groups[-1][0]) <= tol:
            groups[-1][1].append((y, x, text))
        else:
            groups.append((x, [(y, x, text)]))
    lines = []
    for x, items in groups:
        items.sort(key=lambda r: r[0])
        txt = "".join(t for _, _, t in items)
        lines.append((x, items[0][0], items[-1][0], txt))
    return lines


def main(argv):
    reader = PdfReader(argv[1])
    first, last = int(argv[2]), int(argv[3])
    tol = float(argv[4]) if len(argv) > 4 else 3.0
    desc = len(argv) > 5 and argv[5] == "desc"
    for pageno in range(first, last + 1):
        page = reader.pages[pageno - 1]
        runs = glyphs(page)
        print("=" * 118)
        print("PAGE %d runs=%d skew_deg=%.1f" % (pageno, len(runs), rotation_of(runs)))
        lines = cluster_lines(runs, tol)
        if desc:
            lines = list(reversed(lines))
        for x, y0, y1, txt in lines:
            print("x=%8.1f y=%7.1f..%7.1f | %s" % (x, y0, y1, txt))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))