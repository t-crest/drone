# Drone Controller

This repository uses submodules.
Use the option `--recursive` to fetch the submodules when cloning:

    $ git clone --recursive https://github.com/t-crest/drone.git

Alternatively, initialize the submodules after fetching the main repository:

    $ git clone https://github.com/t-crest/drone.git
    $ cd drone
    $ git submodule update --init --recursive

## Build Instructions

First, build the fixed-point arithmetic math library:

    $ cd drone/libfix32math
    $ make

Then, ensure that you have python 3 installed with the following packages: `numpy`, `sympy`, `matplotlib`. This can be done using `pip3`:

```
pip3 numpy, sympy, matplotlib
```

Then, build and simulate the controller:

    $ cd drone/src
    $ make
    $ pasim drone.elf
