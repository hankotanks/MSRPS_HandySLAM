import os, sys, numpy, open3d

if __name__ == '__main__':
    if len(sys.argv) != 5:
        print(f'USAGE: {sys.argv[0]} <path_scene> <path_out> <name_cloud_in> <name_cloud_out>')
        print('  Replace <path_out> with SAME if no output folder was specified in HandySLAM')
        sys.exit(1)

    path_scene = sys.argv[1]
    if not os.path.exists(path_scene):
        print(f'ERROR: {path_scene} does not exist')
        sys.exit(1)

    path_out = sys.argv[2]
    if path_out == 'SAME':
        path_out = path_scene

    name_cloud_in = sys.argv[3]
    path_cloud_in = os.path.join(path_out, name_cloud_in)
    if not os.path.exists(path_cloud_in):
        print(f'ERROR: {path_cloud_in} does not exist')

    name_cloud_out = sys.argv[4]
    path_cloud_out = os.path.join(path_out, name_cloud_out)
    if not os.path.exists(path_cloud_out):
        os.mkdir(path_cloud_out)
    elif os.listdir(path_cloud_out):
        print(f'ERROR: {path_cloud_out} is not empty')
        sys.exit(1)

    path_mesh = os.path.join(path_scene, 'scans', 'mesh_aligned_0.05.ply')
    if not os.path.exists(path_mesh):
        print(f'ERROR: {path_mesh} does not exist')
        sys.exit(1)
    
    bb = open3d.io.read_triangle_mesh(path_mesh).get_axis_aligned_bounding_box()
    
    print('INPUT: <transform> (Ctrl+D to submit):')
    t = numpy.fromstring(sys.stdin.read(), sep = ' ').reshape((4, 4))

    print(end = '\n\n')
    print('Transform:')
    print(t, end = '\n\n')
    
    print('Axis-Aligned Bounding Box:')
    print(f'  minima: {bb.get_min_bound()}')
    print(f'  maxima: {bb.get_max_bound()}', end = '\n\n')

    for filename in os.listdir(path_cloud_in):
        if filename.lower().endswith(".ply"):
            path_ply = os.path.join(path_cloud_in, filename)
            path_ply_out = os.path.join(path_cloud_out, filename)
            print(f'Loading {path_ply}', end = ' ')
            pc = open3d.io.read_point_cloud(path_ply)
            print('[FINISHED]')
            print(f'Transforming {path_ply}', end = ' ')
            pc.transform(t)
            print('[FINISHED]')
            print(f'Cropping {path_ply} to bounding box', end = ' ')
            pc_cropped = pc.crop(bb)
            print('[FINISHED]')
            print(f'Saving {path_ply_out}', end = ' ')
            open3d.io.write_point_cloud(path_ply_out, pc_cropped)
            print('[FINISHED]')
    
    print('Completed ScanNetPostProcess')