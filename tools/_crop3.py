from _crop2 import crop

# PDF page 23 == printed page 16 == TABLE 2 (control travels, cyclic rigging,
# cyclic pitch ranges).  English column is the left half of the table area.
crop('_imgs/page23_0.png', '_imgs/t2_ctl_en.png', 0.45, 1.00, 4, 0.05, 0.50)
crop('_imgs/page23_0.png', '_imgs/t2_ctl_met.png', 0.45, 1.00, 4, 0.50, 0.95)