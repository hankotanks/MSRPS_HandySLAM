import os
import glob
import tqdm

from promptda.utils.io_wrapper import load_image, load_depth, save_depth
from promptda.utils.parallel_utils import parallel_execution

from promptda.promptda import PromptDA

DEVICE = 'cuda'

def main(path_color: str, path_depth: str, path_out: str, width: int):
    files_color = sorted(glob.glob(os.path.join(path_color, '*.png')))
    files_depth = sorted(glob.glob(os.path.join(path_depth, '*.png')))

    if len(files_color) != len(files_depth):
        files_count = min(len(files_color), len(files_depth))
        files_color = files_color[:files_count]
        files_depth = files_depth[:files_count]

    color = parallel_execution(files_color, to_tensor=True, max_size=width, action=load_image, num_processes=32, print_progress=True, desc='Loading RGB imagery')
    depth = parallel_execution(files_depth, to_tensor=True, action=load_depth, num_processes=32, print_progress=True, desc='Loading depth imagery')

    model = PromptDA.from_pretrained("depth-anything/prompt-depth-anything-vitl").to(DEVICE).eval()
    for frame_idx, (curr_color, curr_depth) in tqdm.auto.tqdm(enumerate(zip(color, depth)), desc='Inferring', total=len(color)):
        curr_color, curr_depth = curr_color.to(DEVICE), curr_depth.to(DEVICE)
        pred_depth = model.predict(curr_color, curr_depth)
        save_depth(pred_depth.detach().cpu(), output_path=os.path.join(path_out, os.path.basename(files_depth[frame_idx])), save_vis=False)
    
    return True
    