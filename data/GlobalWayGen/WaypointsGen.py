import math
import csv

def calculate_intermediate_points(start, end, interval):
    """
    计算两点之间的中间点
    :param start: 起点坐标 (x1, y1)
    :param end: 终点坐标 (x2, y2)
    :param interval: 点之间的间隔距离
    :return: 包含(x, y, θ)的列表，θ是朝向角度（弧度）
    """
    x1, y1 = start
    x2, y2 = end
    
    # 计算两点之间的总距离
    dx = x2 - x1
    dy = y2 - y1
    distance = math.sqrt(dx**2 + dy**2)
    
    # 计算方向角度（弧度）
    theta = math.atan2(dy, dx)
    
    # 计算单位向量
    if distance == 0:
        return [(x1, y1, theta)]
    
    unit_x = dx / distance
    unit_y = dy / distance
    
    points = []
    
    # 从起点开始，按间隔距离添加点
    current_dist = 0.0
    while current_dist <= distance:
        x = x1 + unit_x * current_dist
        y = y1 + unit_y * current_dist
        points.append((x, y, theta))
        current_dist += interval
    
    # 确保终点被包含
    if not points or points[-1][:2] != (x2, y2):
        points.append((x2, y2, theta))
    
    return points

def calculate_radial_points(points, d):
    """
    计算径向移动后的点
    :param points: 原始点列表 [(x, y, θ)]
    :param d: 径向移动距离（正数为右侧，负数为左侧）
    :return: 包含(xr, yr, θr)的列表
    """
    radial_points = []
    for x, y, theta in points:
        # 计算法线方向（旋转90度）
        nx = -math.sin(theta)
        ny = math.cos(theta)
        
        # 计算径向移动后的点
        xr = x + nx * d
        yr = y + ny * d
        thetar = theta  # 方向保持不变
        
        radial_points.append((xr, yr, thetar))
    
    return radial_points

def save_to_csv(original_points, radial_points, filename):
    """
    将原始点和径向点保存到CSV文件
    :param original_points: 原始点列表 [(x, y, θ)]
    :param radial_points: 径向点列表 [(xr, yr, θr)]
    :param filename: 输出文件名
    """
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # 写入表头
        writer.writerow(['x_center', 'y_center', 'theta_radians_center', 'theta_degrees_center', 'x', 'y', 'theta_radians', 'theta_degrees' ])
        if len(original_points) != len(radial_points):
            raise ValueError("原始点和径向点的数量不匹配")

        for i in range(len(original_points)):
            x, y, theta = original_points[i]
            xr, yr, thetar = radial_points[i]
            writer.writerow([ xr, yr, thetar, math.degrees(thetar), x, y, theta, math.degrees(theta)])

if __name__ == "__main__":
    # 用户输入
    start_x = 10.0
    start_y = 100.0
    end_x =120
    end_y =120
    interval = 1.0
    d = -1.75
    output_file = "waypoints.csv"
    
    # 计算原始点
    original_points = calculate_intermediate_points((start_x, start_y), (end_x, end_y), interval)
    
    # 计算径向点
    radial_points = calculate_radial_points(original_points, d)
    
    # 保存到CSV
    save_to_csv(original_points, radial_points, output_file)
    
    print(f"已生成 {len(original_points)} 个原始点和 {len(radial_points)} 个径向点，并保存到 {output_file}")