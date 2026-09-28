"""Reconstruct rotated text-layer pages from a PDF into a readable grid.

Usage: python _pdfrot.py <pdf> <first_page> <last_page> [col_width_pts]

Reads only the embedded text layer and the per-text-run transformation matrices
(character geometry is taken from the content stream, no raster analysis).
"""
import math
import sys

from pypdf import PdfReader


def mmul(m, n):
    a = m[0] * n[0] + m[1] * n[2]
    b = m[0] * n[1] + m[1] * n[3]
    c = m[2] * n[0] + m[3] * n[2]
    d = m[2] * n[1] + m[3] * n[3]
    e = m[4] * n[0] + m[5] * n[2] + n[4]
    f = m[4] * n[1] + m[5] * n[3] + n[5]
    return (a, b, c, d, e, f)


def runs_of_page(page):
    runs = []

    def visitor(text, cm, tm, font_dict, font_size):
        if not text or not text.strip():
            return
        m = mmul(cm, tm)
        ang = math.degrees(math.atan2(m[1], m[0]))
        size = font_size or math.hypot(m[2], m[3]) or 10.0
        runs.append((m[4], m[5], ang, size, text.strip()))

    page.extract_text(visitor_text=visitor)
    return runs


def render(runs, col_width):
    # Normalise orientation: rotate every run's origin by -angle so that all
    # text shares one horizontal axis, then place runs on a character grid.
    norm = []
    for x, y, ang, size, text in runs:
        rad = math.radians(ang)
        nx = x * math.cos(rad) + y * math.sin(rad)
        ny = -x * math.sin(rad) + y * math.cos(rad)
        norm.append((ny, nx, size, text, ang))

    norm.sort(key=lambda r: (-round(r[0], 1), r[1]))

    lines = []
    cur_y = None
    cur_items = []
    for ny, nx, size, text, ang in norm:
        if cur_y is None or abs(ny - cur_y) > max(size * 0.6, 1.5):
            if cur_items:
                lines.append((cur_y, cur_items))
            cur_y = ny
            cur_items = []
        cur_items.append((nx, size, text))
    if cur_items:
        lines.append((cur_y, cur_items))

    out = []
    for ny, items in lines:
        row = [" "] * 200
        for nx, size, text in sorted(items):
            col = max(0, min(199, int(round(nx / col_width))))
            for k, ch in enumerate(text):
                if col + k < 200:
                    row[col + k] = ch
        out.append("%9.1f |%s" % (ny, "".join(row).rstrip()))
    return out


def main(argv):
    reader = PdfReader(argv[1])
    first, last = int(argv[2]), int(argv[3])
    col_width = float(argv[4]) if len(argv) > 4 else 4.5
    for pageno in range(first, last + 1):
        page = reader.pages[pageno - 1]
        runs = runs_of_page(page)
        angles = sorted({round(r[2]) for r in runs})
        print("=" * 100)
        print("PAGE %d  rotate=%s  runs=%d  angles=%s" % (pageno, page.get("/Rotate", 0), len(runs), angles))
        for line in render(runs, col_width):
            print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))