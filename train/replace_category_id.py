import glob
import json

path = '/media/gouda/6DB934E957073F8F/datasets/ml2r/train'

for folder in glob.glob(path + '/*'):
    print(folder)

    with open(folder + '/scene_gt_coco.json', "r") as f:
        data = json.load(f)
        data['categories'] = [{'id': 1, 'name': '1', 'supercategory': 'ycbv'}]
        for sample in range(len(data['annotations'])):
            # print(sample)
            # print(data['annotations'][sample]['category_id'])
            data['annotations'][sample]['category_id'] = 1
            # print(data['annotations'][sample]['category_id'])

    with open(folder + '/scene_gt_coco_agnostic.json', "w") as f:
        json.dump(data, f)
