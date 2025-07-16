from lz4.block import decompress

import numpy as np
import imageio as iio

def main(path_depth_bin, path_depth_out):
    height, width, rate = 192, 256, 1
    
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
                data = decompress(data, uncompressed_size=height * width * 2)
                depth = np.frombuffer(data, dtype=np.uint16).reshape(height, width)
            except: return False

            iio.imwrite(f"{path_depth_out}/frame_{frame_idx:06}.png", depth)
            frame_idx += 1
    
    return True