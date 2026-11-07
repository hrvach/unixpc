# CPU

* `fx68k.sv`, `fx68kAlu.sv`, `uaddrPla.sv`: the 68000 core, Jorge Cwik's FX68K (https://github.com/ijor/fx68k). **Changed here, in place, to be a 68010.** The micro and
  nano ROM images (`roms/microrom.mem`, `roms/nanorom.mem`) and the `microToNanoAddr` table in `fx68k.sv` are generated from the original 68000 ROMs by
  `tools/cpu_microcode/`, which says which patches were applied. The rest of the three files is upstream's, in upstream's style, and is not reformatted to the
  style guide of this project (`doc/hdl_style.md`). See the headers of the files and the upstream repository for the licence.
* `cpu_fx68k.sv` is ours: it wraps the core for the board (bus signals, interrupt, function codes).
