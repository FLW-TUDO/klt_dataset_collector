import glob
import os
import json

def main():
    dataset_path = '/home/gouda/segmentation/datasets/ML2R_segmentation_dataset_BOP_format/ml2r/test'
    delete_views = [6,7,18,19] # zero based count
    num_of_views = 20
    for sample in glob.glob(dataset_path + '/*'):
        print(sample)
        # delete images and annotations
        for view in delete_views:
            os.remove(os.path.join(sample, "rgb", f"{view:06}" + ".png"))
            os.remove(os.path.join(sample, "depth", f"{view:06}" + ".png"))
            cloud_file = os.path.join(sample, "cloud", f"{view:06}") + '*'
            cloud_file = glob.glob(cloud_file)[0]
            os.remove(cloud_file)
            try:
                os.remove(os.path.join(sample, "scene_gt_info.json"))
            except:
                pass

            with open(os.path.join(sample,"scene_transformations.json"), "r+") as f:
                data = json.load(f)
                del data[str(view)]
                f.seek(0)
                f.write(json.dumps(data))
                f.truncate()

        # rename all files and json
        all = list(range(20))
        remaining_views = [x for x in all if x not in delete_views]
        for view in remaining_views:
            os.rename(os.path.join(sample, "rgb", f'{view:06}'+'.png'), os.path.join(sample, "rgb", f'{remaining_views.index(view):06}'+'.png'))
            os.rename(os.path.join(sample, "depth", f'{view:06}'+'.png'), os.path.join(sample, "depth", f'{remaining_views.index(view):06}'+'.png'))
            cloud_file = glob.glob(sample + "/cloud/" + f'{view:06}' + '_*.pcd')[0]
            os.rename(cloud_file, cloud_file[:-27] + f'{remaining_views.index(view):06}' + cloud_file[-21:])
            # change numbering of scene_transformations.json
            with open(os.path.join(sample,"scene_transformations.json"), "r+") as f:
                data = json.load(f)
                data[str(remaining_views.index(view))] = data.pop(str(view))
                f.seek(0)
                f.write(json.dumps(data))
                f.truncate()


if __name__ == '__main__':
    main()
