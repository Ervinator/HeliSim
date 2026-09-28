"""Throwaway greps for the TM-73254 / TM-55-1520-210-10 constant hunt."""
import io
import re


def lines(path):
    return io.open(path, encoding='utf-8', errors='replace').read().splitlines()


out = []

# 1. numeric tokens in the cell-per-line TM-73254 dump, with line numbers
pat = re.compile(r'^\s*[-+]?\d*\.\d+\s*$')
for i, ln in enumerate(lines('_tm73254.txt'), 1):
    if pat.match(ln) and i > 1900:
        out.append('NUM54 %4d| %s' % (i, ln.strip()))

# 2. rigging / pitch keywords in the operator's manual
fm = lines(r'reference\TM55-1520-210-10_fulltext.txt')
kw = re.compile(r'rigg|swash|blade angle|blade pitch|pedal|stabilizer bar|mixing|linkage',
                re.I)
for i, ln in enumerate(fm, 1):
    if kw.search(ln):
        out.append('FM %5d| %s' % (i, ln.strip()[:200]))

io.open('_g_out.txt', 'w', encoding='utf-8').write('\n'.join(out))
print('wrote', len(out), 'lines')