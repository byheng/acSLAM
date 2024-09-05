# sudo /usr/local/share/pynq-venv/bin/python LoadBitstream.py *.xsa

import argparse
from pynq import Overlay

parser = argparse.ArgumentParser()
parser.add_argument('bitstream_filename', type=str, help="文件名.xsa or 文件名.bit")

bitstream = parser.parse_args().bitstream_filename

overlay = Overlay(bitstream)
if overlay.is_loaded():
    print(f"Successfully loaded {bitstream}")
