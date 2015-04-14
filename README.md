# kgpkubs3d - Code base for Robocup 3d League #
===========================================================
## Borrowed from: ##
### Sander van Dijk (sgdijk@gmail.com) ###

[![Join the chat at https://gitter.im/KRSSG/kgpkubs3d](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/KRSSG/kgpkubs3d?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)
https://github.com/sgvandijk/libbats

## Fork Maintained by: ##
### Akshay Gupta (akshaysngupta@gmail.com) ###
***

**Dependencies:**

- Eigen 3
- libxml-2.0
- sigc++-2.0
- gtkmm-2.4 (optional; for GTK+ debugger)
- Doxygen (optional; for HTML documentation)
- pdflatex (optional; for PDF manual)

***Ubuntu:***
```
#!bash
sudo apt-get install libeigen3-dev libxml2-dev libsigc++-2.0-dev [libgtkmm-2.4-dev] [doxygen] [texlive-latex-base]
cmake . && make && sudo make install
```

**Documentation:**

If you have `pdflatex` installed while running `cmake`, you
can find the user manual at `docs/manual/libbatsmanual.pdf`. If
you have `doxygen` installed while running `cmake`, HTML based
documentation can be found in `docs/html`.

 * * *

Cheers,
**Team KRSSG**