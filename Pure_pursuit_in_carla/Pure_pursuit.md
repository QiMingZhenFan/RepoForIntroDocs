# Pure Pursuit In Carla

## 实现流程

1. 先决条件
    * 链接Carla并生成车辆

        ```python
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)
        world = client.get_world()
        debug = world.debug

        # 生成车辆
        blueprint_library = world.get_blueprint_library()
        bp = random.choice(blueprint_library.filter('vehicle.audi.etron'))  # 随机选个车型
        spawn_location = carla.Location(93.0,1.0,1.0)
        spawn_rotation = carla.Rotation(0,180,0)
        spawn_transform = carla.Transform(spawn_location,spawn_rotation)
        if bp.has_attribute('color'):   # 随机选个颜色
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)
        vehicle = world.spawn_actor(bp, spawn_transform)
        vehicle_id = vehicle.id
        print('created：', vehicle)
        # 没这个延时车辆生成到（0,0,0）了  -- TODO: reason unknown
        time.sleep(1)
        ```

    * 获取车辆轴距信息
  
        `wheelbase = get_wheelbase(vehicle)`
  
        ```python
        def get_wheelbase(vehicle):
        # 获取轴距
        physics_control = vehicle.get_physics_control()
        wheels = physics_control.wheels
        wheels_pos = []
        wheels_pos.append(wheels[0].position)
        wheels_pos.append(wheels[1].position)
        wheels_pos.append(wheels[2].position)
        wheels_pos.append(wheels[3].position)
        wheelbase = pow(wheels_pos[2].x-wheels_pos[0].x,2) \
                    + pow(wheels_pos[2].y-wheels_pos[0].y,2) \
                    + pow(wheels_pos[2].z-wheels_pos[0].z,2)
        # 这个wheel.position只是个Vector3D类型，没有说单位是多少
        # 认为单位为厘米（cm）
        return pow(wheelbase,0.5)
        ```

    * 给定油门以维持车速

        ```python
        # 控制油门、前轮转角、刹车
        my_vehiclecontrol = carla.VehicleControl(throttle = 0.5)
        vehicle.apply_control(my_vehiclecontrol)
        ```

2. 读取csv文件中的waypoint,适配自定义的地图格式类型：

    行号 | 含义
    ---------|----------
    第一行 | 道路id
    第二行 | 道路长度
    第三行 | 是否双向,1:双向, 0:单向
    第四行 | 前序
    第五行 | 后序
    后续所有行 | x,y,z,四元数

    ```python
    # -------------------------------------------
    # @note 读取csv文件相关
    # -------------------------------------------
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
            j = 6   # 从第6项开始是坐标等信息
            while j < len(sub_path):
                path.append(sub_path[j])
                j += 1
        path.pop(0) # 上面二维列表初始化导致最开始元素是一个的一维的空列表
        return path

    def get_path_main():
        filepath = []
        filepath.append('path/to/lane_0.csv')
        filepath.append('path/to/lane_1.csv')
        path = get_path(*filepath)
        return path

    def draw_target_path(debug,path):
        index = 0
        for i in path:
            location1 = carla.Location(i[0],i[1],i[2])

            if index != 0:
                draw_location_union(debug, location0, location1, green, 60)
            index += 1
            location0 = location1
        pass
    ```

3. 获取车辆当前状态，用于后续更新控制量

    ```python
    # -------------------------------------------
    # @note 利用snapshot更新车辆的状态参数
    # -------------------------------------------
    def get_current_state(world, vehicle_id):
        current_snapshot = world.get_snapshot()
        ActorSnapshot = current_snapshot.find(vehicle_id)
        if ActorSnapshot != None:
            # 获得位置信息
            Actor_transform = ActorSnapshot.get_transform()
            # 获得速度（标量）
            Actor_v = ActorSnapshot.get_velocity().x ** 2 + \
                    ActorSnapshot.get_velocity().y ** 2 + \
                    ActorSnapshot.get_velocity().z ** 2
            Actor_v = pow(Actor_v, 0.5)

            return Actor_transform,Actor_v
        else:
            print('cannot get this actor!')
    # -------------------------------------------
    ```

4. Pure Pursuit核心代码

    ```python
    # -------------------------------------------
    # @note Pure Pursuit核心代码
    # -------------------------------------------
    k = 0.1  # 前视距离系数
    Lfc = 2.0  # 前视距离
    Kp = 1.0  # 速度P控制器系数
    dt = 0.1  # 时间间隔，单位：s
    L = 2.9  # 车辆轴距，单位：m

    import math

    def pure_pursuit_control(transform, v, cx, cy, pind):
        ind = calc_target_index(transform, cx, cy, v)

        if pind >= ind:  # 上一次最近的在这一次最近的后面
            ind = pind

        if ind < len(cx):
            tx = cx[ind]
            ty = cy[ind]
        else:  # 最后一个点
            tx = cx[-1]
            ty = cy[-1]
            ind = len(cx) - 1

        alpha = math.atan2(ty - transform.location.y, tx - transform.location.x) - math.radians(transform.rotation.yaw)

        if v < 0:  # backward
            alpha = math.pi - alpha

        Lf = k * v + Lfc
        # 以弧度为单位
        delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

        return delta, ind


    def calc_target_index(transform, cx, cy, velocity):
        # 搜索最临近的路点
        dx = [transform.location.x - icx for icx in cx]
        dy = [transform.location.y - icy for icy in cy]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))
        L = 0.0

        Lf = k * velocity + Lfc
        while Lf > L and (ind + 1) < len(cx):
            dx = cx[ind + 1] - cx[ind]
            dy = cy[ind + 1] - cy[ind]
            L += math.sqrt(dx ** 2 + dy ** 2)
            ind += 1
        return ind
   ```

---

## waypoint可视化展示（便于debug）

```python
red = carla.Color(255, 0, 0)
green = carla.Color(0, 255, 0)
blue = carla.Color(47, 210, 231)
cyan = carla.Color(0, 255, 255)
yellow = carla.Color(255, 255, 0)
orange = carla.Color(255, 162, 0)
white = carla.Color(255, 255, 255)

def draw_transform(debug, trans, col=carla.Color(255, 0, 0), lt=-1):
    debug.draw_arrow(
        trans.location, trans.location + trans.get_forward_vector(),
        thickness=0.05, arrow_size=0.1, color=col, life_time=lt)


def draw_waypoint_union(debug, w0, w1, color=carla.Color(255, 0, 0), lt=5):
    debug.draw_line(
        w0.transform.location + carla.Location(z=0.25),
        w1.transform.location + carla.Location(z=0.25),
        thickness=0.1, color=color, life_time=lt, persistent_lines=False)
    debug.draw_point(w1.transform.location + carla.Location(z=0.25), 0.1, color, lt, False)

# location版本，不需要输入waypoint
def draw_location_union(debug, location0, location1, color=carla.Color(255, 0, 0), lt=5):
    debug.draw_line(
        location0 + carla.Location(z=0.25),
        location1 + carla.Location(z=0.25),
        thickness=0.05, color=color, life_time=lt, persistent_lines=False)
    debug.draw_point(location1 + carla.Location(z=0.25), 0.08, color, lt, False)

```

---

## TODO

- [ ] 生成vehicle最好使用try版函数
- [ ] 更换map的时候不延时或者延时时间短会连接不上服务器报错
- [ ] 删除所有actor时不可循环删除，应该`client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])`
- [ ] vehiclecontrol中的steer变量是归一化后的值，但是Carla中并未给出归一化系数，此处设为pi/3(待议)

---
