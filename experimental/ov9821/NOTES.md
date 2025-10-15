# libcamera test for RPI5 and OV9281

hacking up libcamera.org's simple-cam example to work with grayscale (16-bit sadly)
raw data.

will need libcamera dev

```bash
apt install libcamera-dev
```
also needs meson and ninja
```bash
apt install meson ninja-build
```


build with 
```bash
meson build
cd build
ninja
```

