import csv

def read_csv(str):
    with open(str, 'r') as file:
        content = csv.reader(file,dialect='excel')
        data = [[]]
        index = 0   # 从第一行开始，第0行的list作空处理
        for line in content:
            data.append([])
            index += 1
            for element in line:
                data[index].append(float(element))
    return data


def get_path(*filepath):
    path = [[]]
    for i in filepath:
        sub_path = read_csv(i)
        # TODO:二维列表，只能这样循环生成吗
        j = 6   # 从第6项开始是坐标等信息
        while j < len(sub_path):
            path.append(sub_path[j])
            j += 1
    path.pop(0) # 上面二维列表初始化导致最开始元素是一个的一维的空列表
    return path


def main():
    filepath = []
    filepath.append('/home/mrk/Lab_file/carla_simu/lane_0.csv')
    filepath.append('/home/mrk/Lab_file/carla_simu/lane_1.csv')
    path = get_path(*filepath)
    for i in path:
        print(i)
    pass


if __name__ == '__main__':
    main()
