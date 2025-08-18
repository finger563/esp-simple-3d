#!/bin/bash

# Convert a bmp to a jpg and resize it to 256x256 pixels, replacing the bmp extension with jpg.
find . -maxdepth 1 -iname "*.bmp" | xargs -L1 -I{} sh -c 'magick "{}" -resize 128x128 "../fs/textures/${0%.bmp}.jpg"' {}
