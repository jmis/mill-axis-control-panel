# Build & Upload
Make sure your user is in the dialout group.

```
docker run --rm --device=/dev/ttyUSB0 -e HOME=/project -u $UID -v /dev:/dev -v $PWD:/project -w /project petewall/platformio run --project-dir /project -t upload --environment esp32dev
```