# water-matrix

Water simulation for RP2350-matrix module

# Steps

```
git clone https://github.com/poconoco/water-matrix.git ~/water-matrix
cd ~/water-matrix
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make -j12
```

# Reboot to boot mode

Either by resetting with boot button pressed, or when connected to usb, assuming board usb serial got mapped to `/dev/ttyACM0`:

```
stty -F /dev/ttyACM0 1200
```

# Flash the new firmware

Assuming Pi Pico was mounted automatically at `/media/$USER/RP2350/`:

```
cd ~/water-matrix/build
cp main.uf2 /media/$USER/RP2350/
```

# Quick rebuild + reflash

When you did any changes to the source, you can use a one-line command to rebuild and flash it, should be run from the `~/water-matrix/build` workdir:

```
make -j12 && stty -F /dev/ttyACM0 1200 && sleep 5 && cp main.uf2 /media/$USER/RP2350/
```
