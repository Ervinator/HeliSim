# Tools

The scripts that dug the constants out of the reference PDFs.  They are kept for
provenance: when a number in `aerodynamics.py` cites TM-73254's table 3, this is
how it was read.

**They are not part of the project and not covered by `requirements.txt`.**  Two
of them import outside the standard library -

    pypdf    (the text layer, page objects, content streams)
    Pillow   (the page image crops)

- and they were written to be run from the project root, which is where the PDFs
and the dumps used to live.  They read the reference PDFs by relative path
(`reference\NASA-TM-73254_UH-1H-force-and-moment-model.pdf`) and they write
their output into the current directory (`_pg21.txt`, `_imgs/page23_0.png`, and
so on) rather than next to themselves, so a re-run from `tools/` needs those
paths edited, and a re-run from the root will recreate the dump files there.
`_crop3.py` imports `_crop2`, so those two need to stay together.

## Finding a page

| script | what it does |
| --- | --- |
| `_pdfscan.py` | locate the pages whose text layer contains a needle, and report their geometry (`--dump PAGE`, `--layout`) |
| `_pdfmap.py` | map a PDF's text layer page by page: first lines and character count |
| `_pagemap.py` | map every page of the TM-73254 copy: text length, embedded images, and any TABLE / CONTROL TRAVELS / LINKAGE hits |
| `_p2.py` | map PDF pages of the TM-73254 copy to report tables by number |
| `_pdfraw.py` | dump the raw text-layer runs - x, y, text - for one page |

## Recovering a table the OCR mangled

| script | what it does |
| --- | --- |
| `_pdfrot.py` | reconstruct rotated text-layer pages into a readable grid |
| `_pdfrot2.py` | cluster per-glyph runs into rotated table lines |
| `_cs.py` | diagnose the mangled pages at the XObject and text-matrix level |
| `_dump_pages.py` | dump the full extractable text of the TM-73254 table pages (PDF pages 21 to 28) |
| `_g.py` | the throwaway greps from the constant hunt, kept as a record of what was looked for |

## Getting an image a human can read

| script | what it does |
| --- | --- |
| `_img.py` | dump the images embedded in the mangled table pages |
| `_png.py` | export the scanned page images of the table pages as PNG |
| `_png3.py` | export PDF pages 29 to 32 as PNG plus small overviews |
| `_png4.py` | export PDF pages 12 to 14 (printed 5 to 7), where the linkage constants c1 to c6 and the stabilizer bar constants K_B and tau_B are defined |
| `_tiff.py` | wrap the CCITT page scans as TIFF for the Windows imaging component |
| `_crop.py`, `_crop2.py`, `_crop3.py` | crop a band out of a page image and magnify it; the crop table 3's linkage constants came off |
| `_ov.py` | overlay a small crop on its full page, to see where a band sits |

The images they produced are in `../attic/_imgs/`, and the per-page text dumps
they wrote are in `../attic/` with the rest of the scratch.  Nothing here is
needed to run or test HeliSim.
