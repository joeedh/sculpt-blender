To generate a new blue noise mask, edit config.json (dimen must be a power of 2)
then run

./genmask.sh

It will install the bluegen npm package (if necassary), then run it to
generate the mask.  The mask is generated with a fairly basic 
void-cluster implementation (bluegen is actually a research project of mine,
it can generate lots of different types of blue noise masks but we just
want a simple one here).

genmask will generate two files, mask64.png and masksrc.h.  Masksrc.h will contain 
a C source encoding of the mask.  To use, copy the large array to:

intern/cycles/util/utils_bluenoise_mask.cpp

And the defines to:

intern/cycles/kernel/kernel_bluenoise_mask.h

Note that we don't use the generate sampling function.