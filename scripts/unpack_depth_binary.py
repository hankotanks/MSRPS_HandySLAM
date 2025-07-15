from tqdm import tqdm

import subprocess
import sys
import os

import numpy as np
import imageio as iio
import zlib
import lz4.block as lz4

def main(path_depth_bin, path_depth_out):
    height, width, rate = 192, 256, 1
    
    try:
        with open(path_depth_bin, 'rb') as infile:
            data = infile.read()
            data = zlib.decompress(data, wbits=-zlib.MAX_WBITS)
            depth = np.frombuffer(data, dtype=np.float32).reshape(-1, height, width)

        for frame_idx in tqdm(range(0, depth.shape[0], rate), desc='decode_depth'):
            iio.imwrite(f"{path_depth_out}/frame_{frame_idx:06}.png", depth.astype(np.uint16))

    except:
        frame_idx = 0
        with open(path_depth_bin, 'rb') as infile:
            while True:
                size = infile.read(4)
                if len(size) == 0: break

                size = int.from_bytes(size, byteorder='little')
                if frame_idx % rate != 0:
                    infile.seek(size, 1)
                    frame_idx += 1
                    continue

                data = infile.read(size)
                try:
                    data = lz4.decompress(data, uncompressed_size=height * width * 2)
                    depth = np.frombuffer(data, dtype=np.uint16).reshape(height, width)
                except:
                    data = zlib.decompress(data, wbits=-zlib.MAX_WBITS)
                    depth = np.frombuffer(data, dtype=np.float32).reshape(height, width)
                    depth = (depth * 1000).astype(np.uint16)

                iio.imwrite(f"{path_depth_out}/frame_{frame_idx:06}.png", depth)
                frame_idx += 1