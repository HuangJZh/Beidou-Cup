# FAIM
Fusion Autonomous Integrity Monitoring

# Dependencies
Sophus
Eigen
OpenCV

Check Makefile for their minimum required version.

# MIND YOURSELF
Currently, this is a stereo VIO operating with stored data EuRoC MH03.

# Usage

The code was written and edited in vim in Ubuntu.

## 1 Compile
This code was compiled in Ubuntu 16.04.5 using Makefile in terminal
```
mkdir build
cd build
make -j $(nproc)
```

Three files will be generated: libfaim.so, gps-sdr, gps-gse

### libfaim.so
self-explaining.

### gps-sdr
executable. Forgive me. This code was developed using gps-sdr so the name is retained.

### gps-gse
Graphical User Interface (GUI). For now, you don't care.

## 2 Run
Under the 'build' directory, run
```
./gps-sdr
```

Optionally, if you're interesetd in gps-gse, then open a new terminal, run
```
./gps-gse
```

## 3 Evaluate
check evaluation/evaluateEuRoC_MH_03.bash


# Contribution
## Step 1 fork this repo
Ready to edit

## Step 2 Edit the cloned source code files
Ready to commit

## Step 3 Merge
Pull request
And I will review the changes and determine whether to merge your code:-D
