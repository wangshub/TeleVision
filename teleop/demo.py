import math

def quaternion_to_euler(q):
    x, y, z, w = q
    
    # 计算欧拉角
    r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    p = math.asin(2 * (w * y - z * x))
    y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    
    return math.degrees(r), math.degrees(p), math.degrees(y)

# 示例
q = (0.0, -0.7, 0.0, 0.7)  
r, p, y = quaternion_to_euler(q)
print("欧拉角 (r, p, y):", r, p, y)

q = (0.5213417349044974, -0.5213436499031358, -0.47770559480524005, 0.477703840097834,)  
r, p, y = quaternion_to_euler(q)
print("欧拉角 (r, p, y):", r, p, y)