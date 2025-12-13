#!/usr/bin/env python3
"""
Convert JPEG image to BMP format for use with tinybmp library.

Usage: python3 convert_jpg_to_bmp.py input.jpg output.bmp
"""

import sys

from PIL import Image


def convert_jpg_to_bmp(input_jpg, output_bmp):
    """Convert JPEG to BMP format."""
    # Open image
    img = Image.open(input_jpg)

    # Convert to RGB if necessary (remove alpha channel)
    if img.mode != "RGB":
        img = img.convert("RGB")

    width, height = img.size
    print(f"Image size: {width}x{height}")
    print(f"Image mode: {img.mode}")

    # Save as BMP
    img.save(output_bmp, "BMP")

    print(f"Converted {input_jpg} to {output_bmp}")

    # Get file size
    import os

    file_size = os.path.getsize(output_bmp)
    print(f"BMP file size: {file_size:,} bytes")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 convert_jpg_to_bmp.py input.jpg output.bmp")
        sys.exit(1)

    input_jpg = sys.argv[1]
    output_bmp = sys.argv[2]

    convert_jpg_to_bmp(input_jpg, output_bmp)
