import streamlit as st
import pandas as pd
import time
import folium
from streamlit_folium import st_folium
import math
import json
import os
from datetime import datetime

st.set_page_config(layout="wide", page_title="无人机监测系统")

# ==================== 坐标转换函数 ====================
pi = 3.1415926535897932384626
a = 6378245.0
ee = 0.00669342162296594323

def _transform_lat(lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + \
        0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 * math.sin(2.0 * lng * pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * pi) + 40.0 * math.sin(lat / 3.0 * pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * pi) + 320 * math.sin(lat * pi / 30.0)) * 2.0 / 3.0
    return ret

def _transform_lng(lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + \
        0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 * math.sin(2.0 * lng * pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * pi) + 40.0 * math.sin(lng / 3.0 * pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lng / 12.0 * pi) + 300.0 * math.sin(lng / 30.0 * pi)) * 2.0 / 3.0
    return ret

def out_of_china(lng, lat):
    return not (72.004 <= lng <= 137.8347 and 0.8293 <= lat <= 55.8271)

def wgs84_to_gcj02(lng, lat):
    if out_of_china(lng, lat):
        return lng, lat
    dlat = _transform_lat(lng - 105.0, lat - 35.0)
    dlng = _transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * pi)
    return lng + dlng, lat + dlat

def gcj02_to_wgs84(lng, lat):
    if out_of_china(lng, lat):
        return lng, lat
    dlat = _transform_lat(lng - 105.0, lat - 35.0)
    dlng = _transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * pi)
    return lng - dlng, lat - dlat

# ==================== 几何辅助函数（修正经纬度偏移）====================
def point_to_segment_distance(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0, min(1, t))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.hypot(px - proj_x, py - proj_y)

def get_closest_point_on_segment(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return x1, y1
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0, min(1, t))
    return x1 + t * dx, y1 + t * dy

def perpendicular_point(px, py, x1, y1, x2, y2, offset_meters, direction='left'):
    """
    计算从点 (px, py) 沿航线法线方向偏移 offset_meters 米后的点
    偏移量正确转换为经纬度增量
    """
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)
    if length == 0:
        return px + offset_meters, py + offset_meters

    # 单位方向向量
    ux = dx / length
    uy = dy / length

    # 法线方向（左手法则：(-uy, ux) 为左侧）
    perp_x = -uy
    perp_y = ux
    if direction == 'right':
        perp_x = uy
        perp_y = -ux

    # 将偏移量（米）转换为经纬度偏移
    # 纬度方向：1度 ≈ 111320 米
    # 经度方向：1度 ≈ 111320 * cos(lat_rad) 米，这里取中心纬度
    center_lat = py  # 使用当前点的纬度
    lat_rad = math.radians(center_lat)
    meters_per_deg_lat = 111320.0
    meters_per_deg_lng = 111320.0 * math.cos(lat_rad)

    # 偏移向量在经纬度坐标系下的分量
    delta_lng = offset_meters * perp_x / meters_per_deg_lng
    delta_lat = offset_meters * perp_y / meters_per_deg_lat

    return px + delta_lng, py + delta_lat
    
def calculate_avoidance_waypoints(start, end, obstacles, flight_height, safe_radius, strategy, bypass_offset):
    # 1. 筛选威胁障碍物，并构建膨胀后的多边形顶点
    threatening_polygons = []
    for obs in obstacles:
        if obs['height'] >= flight_height:
            coords = obs['coords']
            # 计算中心
            center_lng = sum(c[0] for c in coords) / len(coords)
            center_lat = sum(c[1] for c in coords) / len(coords)
            
            # 构建膨胀多边形（向外推 safe_radius 米）
            buffered_coords = []
            n = len(coords)
            for i in range(n):
                prev = coords[(i-1) % n]
                curr = coords[i]
                next_coord = coords[(i+1) % n]
                
                # 当前顶点的两条边
                dx1 = curr[0] - prev[0]
                dy1 = curr[1] - prev[1]
                dx2 = next_coord[0] - curr[0]
                dy2 = next_coord[1] - curr[1]
                
                len1 = math.hypot(dx1, dy1)
                len2 = math.hypot(dx2, dy2)
                if len1 == 0 or len2 == 0:
                    continue
                
                # 两条边的法向量（垂直于边，指向外侧需要基于顶点角度判断）
                nx1 = -dy1 / len1  # 左侧法向量
                ny1 = dx1 / len1
                nx2 = -dy2 / len2
                ny2 = dx2 / len2
                
                # 合并法向量（平均方向），并归一化
                nx = (nx1 + nx2) / 2
                ny = (ny1 + ny2) / 2
                nl = math.hypot(nx, ny)
                if nl > 0:
                    nx /= nl
                    ny /= nl
                
                # 转换为经纬度偏移
                center_lat_rad = math.radians(center_lat)
                meters_per_deg_lat = 111320.0
                meters_per_deg_lng = 111320.0 * math.cos(center_lat_rad)
                
                expand_lng = curr[0] + (safe_radius * nx) / meters_per_deg_lng
                expand_lat = curr[1] + (safe_radius * ny) / meters_per_deg_lat
                buffered_coords.append((expand_lng, expand_lat))
            
            threatening_polygons.append({
                'coords': buffered_coords,
                'center': (center_lng, center_lat),
                'height': obs['height']
            })
    
    if strategy == 'direct' or not threatening_polygons:
        return [start, end]
    
    # 2. 线段与多边形的相交检测（使用分离轴定理）
    def line_intersects_polygon(p1, p2, polygon_coords):
        # 先检查线段端点是否在多边形内
        def point_in_polygon(px, py, poly):
            inside = False
            j = len(poly) - 1
            for i in range(len(poly)):
                if ((poly[i][1] > py) != (poly[j][1] > py)) and \
                   (px < (poly[j][0] - poly[i][0]) * (py - poly[i][1]) / (poly[j][1] - poly[i][1]) + poly[i][0]):
                    inside = not inside
                j = i
            return inside
        
        if point_in_polygon(p1[0], p1[1], polygon_coords) or point_in_polygon(p2[0], p2[1], polygon_coords):
            return True
        
        # 检查线段与多边形每条边的相交
        for i in range(len(polygon_coords)):
            j = (i + 1) % len(polygon_coords)
            if lines_intersect(p1, p2, polygon_coords[i], polygon_coords[j]):
                return True
        return False
    
    def lines_intersect(a1, a2, b1, b2):
        def ccw(A, B, C):
            return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
        return ccw(a1, b1, b2) != ccw(a2, b1, b2) and ccw(a1, a2, b1) != ccw(a1, a2, b2)
    
    # 3. 找到线段与多边形的交点（用于生成绕行点）
    def get_line_polygon_intersections(p1, p2, polygon_coords):
        intersections = []
        for i in range(len(polygon_coords)):
            j = (i + 1) % len(polygon_coords)
            inter = line_intersection_point(p1, p2, polygon_coords[i], polygon_coords[j])
            if inter:
                intersections.append(inter)
        # 按离 p1 的距离排序
        intersections.sort(key=lambda p: math.hypot(p[0]-p1[0], p[1]-p1[1]))
        return intersections
    
    def line_intersection_point(a1, a2, b1, b2):
        x1, y1 = a1
        x2, y2 = a2
        x3, y3 = b1
        x4, y4 = b2
        
        denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
        if abs(denom) < 1e-12:
            return None
        
        t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / denom
        u = -((x1-x2)*(y1-y3) - (y1-y2)*(x1-x3)) / denom
        
        if 0 <= t <= 1 and 0 <= u <= 1:
            ix = x1 + t * (x2 - x1)
            iy = y1 + t * (y2 - y1)
            return (ix, iy)
        return None
    
    # 4. 主逻辑：逐个处理障碍物，沿着多边形边缘绕行
    current_path = [start]
    remaining_point = end
    
    for poly_data in threatening_polygons:
        poly_coords = poly_data['coords']
        
        # 检查从 current_path[-1] 到 remaining_point 是否穿过该多边形
        if not line_intersects_polygon(current_path[-1], remaining_point, poly_coords):
            continue
        
        # 找到进入点和离开点
        intersections = get_line_polygon_intersections(current_path[-1], remaining_point, poly_coords)
        if len(intersections) < 2:
            # 如果无法找到两个交点，使用中心点作为参考
            center = poly_data['center']
            closest = get_closest_point_on_segment(center[0], center[1], 
                                                   current_path[-1][0], current_path[-1][1], 
                                                   remaining_point[0], remaining_point[1])
            # 生成几个绕行点
            for t in [0.3, 0.5, 0.7]:
                inter_x = current_path[-1][0] + (remaining_point[0] - current_path[-1][0]) * t
                inter_y = current_path[-1][1] + (remaining_point[1] - current_path[-1][1]) * t
                if strategy == 'left':
                    pt = perpendicular_point(inter_x, inter_y,
                                            current_path[-1][0], current_path[-1][1],
                                            remaining_point[0], remaining_point[1],
                                            bypass_offset, 'left')
                else:
                    pt = perpendicular_point(inter_x, inter_y,
                                            current_path[-1][0], current_path[-1][1],
                                            remaining_point[0], remaining_point[1],
                                            bypass_offset, 'right')
                current_path.append(pt)
        else:
            # 有明确交点，沿多边形边缘生成绕行点
            entry_point = intersections[0]
            exit_point = intersections[-1]
            
            # 找到多边形上在 entry 和 exit 之间的顶点序列
            entry_idx = -1
            exit_idx = -1
            min_entry_dist = float('inf')
            min_exit_dist = float('inf')
            
            for i, coord in enumerate(poly_coords):
                d_entry = math.hypot(coord[0]-entry_point[0], coord[1]-entry_point[1])
                d_exit = math.hypot(coord[0]-exit_point[0], coord[1]-exit_point[1])
                if d_entry < min_entry_dist:
                    min_entry_dist = d_entry
                    entry_idx = i
                if d_exit < min_exit_dist:
                    min_exit_dist = d_exit
                    exit_idx = i
            
            # 决定沿多边形的哪个方向走（顺时针或逆时针）
            if strategy == 'left':
                # 左绕行：取逆时针方向的顶点
                step = 1
            elif strategy == 'right':
                step = -1
            else:
                # 最佳策略：选较短的路径
                n_verts = len(poly_coords)
                dist_forward = 0
                dist_backward = 0
                i = entry_idx
                while i != exit_idx:
                    j = (i + 1) % n_verts
                    dist_forward += math.hypot(poly_coords[j][0]-poly_coords[i][0], poly_coords[j][1]-poly_coords[i][1])
                    i = j
                i = entry_idx
                while i != exit_idx:
                    j = (i - 1) % n_verts
                    dist_backward += math.hypot(poly_coords[j][0]-poly_coords[i][0], poly_coords[j][1]-poly_coords[i][1])
                    i = j
                step = 1 if dist_forward <= dist_backward else -1
            
            # 生成绕行航点（取多边形顶点，适当外推）
            current_path.append(entry_point)
            
            i = entry_idx
            while True:
                i = (i + step) % len(poly_coords)
                if i == exit_idx:
                    break
                # 将多边形顶点再往外推一点
                pt = poly_coords[i]
                dx = pt[0] - poly_data['center'][0]
                dy = pt[1] - poly_data['center'][1]
                dist = math.hypot(dx, dy)
                if dist > 0:
                    push_factor = 1.1  # 再推10%
                    new_lng = poly_data['center'][0] + dx * push_factor
                    new_lat = poly_data['center'][1] + dy * push_factor
                    current_path.append((new_lng, new_lat))
                else:
                    current_path.append(pt)
            
            current_path.append(exit_point)
    
    # 确保终点在路径中
    if current_path[-1] != end:
        current_path.append(end)
    
    return current_path
# ==================== 初始化 Session State ====================
if "heartbeats" not in st.session_state:
    st.session_state.heartbeats = []
    st.session_state.last_time = time.time()
    st.session_state.running = False
if "coords_a" not in st.session_state:
    st.session_state.coords_a = {"lat": 32.230500, "lon": 118.748500}
if "coords_b" not in st.session_state:
    st.session_state.coords_b = {"lat": 32.238000, "lon": 118.754000}
if "flight_height" not in st.session_state:
    st.session_state.flight_height = 50
if "safe_radius" not in st.session_state:
    st.session_state.safe_radius = 5.0
if "bypass_offset" not in st.session_state:
    st.session_state.bypass_offset = 5.0
if "coord_system" not in st.session_state:
    st.session_state.coord_system = "GCJ-02 (高德/腾讯)"
if "page" not in st.session_state:
    st.session_state.page = "飞行监控"
if "obstacles" not in st.session_state:
    st.session_state.obstacles = []
if "avoidance_strategy" not in st.session_state:
    st.session_state.avoidance_strategy = "best"
if "pending_polygon" not in st.session_state:
    st.session_state.pending_polygon = None
if "drawn_polygon" not in st.session_state:
    st.session_state.drawn_polygon = []

CONFIG_FILE = "obstacle_config.json"

def load_obstacles():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                data = json.load(f)
            if "obstacles" in data:
                st.session_state.obstacles = data["obstacles"]
                st.success(f"已加载 {len(data['obstacles'])} 个障碍物")
        except Exception as e:
            st.error(f"加载失败: {e}")

def save_obstacles():
    try:
        with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
            json.dump({"obstacles": st.session_state.obstacles}, f, ensure_ascii=False, indent=2)
        st.success("障碍物配置已保存到文件")
    except Exception as e:
        st.error(f"保存失败: {e}")

# ==================== 侧边栏导航 ====================
with st.sidebar:
    st.title("🚁 导航")
    page = st.radio("功能页面", ["飞行监控", "航线规划"])
    st.session_state.page = page

# ==================== 创建地图函数 ====================
def create_complete_map(lat_a, lon_a, lat_b, lon_b, obstacles, flight_height, safe_radius, waypoints):
    center_lat = (lat_a + lat_b) / 2
    center_lon = (lon_a + lon_b) / 2
    m = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=17,
        tiles='https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德卫星地图'
    )

    folium.PolyLine(
        locations=[[lat_a, lon_a], [lat_b, lon_b]],
        color='gray',
        weight=3,
        opacity=0.5,
        dash_array='5,5',
        tooltip='原始航线'
    ).add_to(m)

    folium.PolyLine(
        locations=[(p[1], p[0]) for p in waypoints],
        color='red',
        weight=5,
        opacity=0.8,
        tooltip='规划航线'
    ).add_to(m)

    for i, (lng, lat) in enumerate(waypoints):
        folium.CircleMarker(
            location=[lat, lng],
            radius=4,
            color='blue' if i in (0, len(waypoints)-1) else 'orange',
            fill=True,
            popup=f'航点{i}'
        ).add_to(m)

    folium.Marker(
        location=[lat_a, lon_a],
        popup='起点A',
        icon=folium.Icon(color='green', icon='play', prefix='fa')
    ).add_to(m)
    folium.Marker(
        location=[lat_b, lon_b],
        popup='终点B',
        icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')
    ).add_to(m)

    for obs in obstacles:
        polygon_coords = [[coord[1], coord[0]] for coord in obs["coords"]]
        folium.Polygon(
            locations=polygon_coords,
            color='orange',
            fill=True,
            fill_color='orange',
            fill_opacity=0.4,
            weight=2,
            tooltip=f"{obs['name']} (高{obs['height']}m)"
        ).add_to(m)
        center = [sum(c[1] for c in obs["coords"])/len(obs["coords"]),
                  sum(c[0] for c in obs["coords"])/len(obs["coords"])]
        folium.Marker(
            location=[center[0], center[1]],
            icon=folium.DivIcon(
                html=f'<div style="font-size: 12px; font-weight: bold; color: #ff6600;">{obs["height"]}m</div>'
            )
        ).add_to(m)

    folium.Marker(
        location=[center_lat, center_lon],
        icon=folium.DivIcon(html=f'<div style="background:white; padding:2px 6px; border-radius:15px; border:1px solid red;">✈️ 高度:{flight_height}m | 半径:{safe_radius}m</div>')
    ).add_to(m)

    draw = folium.plugins.Draw(
        draw_options={
            'polyline': False,
            'rectangle': False,
            'circle': False,
            'marker': False,
            'circlemarker': False,
            'polygon': True
        },
        edit_options={'edit': True}
    )
    draw.add_to(m)
    return m

# ==================== 航线规划页面 ====================
if st.session_state.page == "航线规划":
    st.title("🗺️ 航线规划 + 障碍物圈选")

    with st.sidebar:
        st.divider()
        st.header("🎮 坐标系设置")
        coord_system = st.selectbox(
            "输入坐标系",
            ["GCJ-02 (高德/腾讯)", "WGS-84 (GPS)"],
            index=0 if "GCJ-02" in st.session_state.coord_system else 1
        )
        st.session_state.coord_system = coord_system
        is_gcj02 = "GCJ-02" in coord_system

        st.divider()
        st.header("📍 起点 A")
        lat_a_input = st.number_input("纬度 A", value=st.session_state.coords_a["lat"], format="%.6f")
        lon_a_input = st.number_input("经度 A", value=st.session_state.coords_a["lon"], format="%.6f")
        
        st.header("📍 终点 B")
        lat_b_input = st.number_input("纬度 B", value=st.session_state.coords_b["lat"], format="%.6f")
        lon_b_input = st.number_input("经度 B", value=st.session_state.coords_b["lon"], format="%.6f")

        st.divider()
        st.header("✈️ 飞行参数")
        flight_height = st.slider("飞行高度 (m)", 20, 100, st.session_state.flight_height)
        st.session_state.flight_height = flight_height
        safe_radius = st.number_input("安全半径 (m)", min_value=1.0, max_value=50.0, value=st.session_state.safe_radius, step=1.0)
        st.session_state.safe_radius = safe_radius
        bypass_offset = st.slider("绕行偏移量 (米) - 仅水平绕行几米", min_value=2.0, max_value=20.0, value=st.session_state.bypass_offset, step=1.0)
        st.session_state.bypass_offset = bypass_offset

        st.divider()
        st.header("🔄 避障策略")
        strategy = st.radio(
            "选择绕行方式",
            options=['direct', 'left', 'right', 'best'],
            format_func=lambda x: { 
                'direct': '直接飞 (高度足够时)', 
                'left': '向左绕行', 
                'right': '向右绕行', 
                'best': '最佳航线' 
            }[x],
            index=['direct', 'left', 'right', 'best'].index(st.session_state.avoidance_strategy)
        )
        st.session_state.avoidance_strategy = strategy

        st.divider()
        st.subheader("🗂️ 障碍物持久化")
        col1, col2 = st.columns(2)
        with col1:
            if st.button("💾 保存障碍物", use_container_width=True):
                save_obstacles()
        with col2:
            if st.button("📂 加载障碍物", use_container_width=True):
                load_obstacles()
        if st.button("🗑️ 清除全部障碍物", use_container_width=True):
            st.session_state.obstacles = []
            st.session_state.drawn_polygon = []
            st.success("已清除所有障碍物")
        
        st.divider()
        st.subheader("➕ 添加障碍物（多边形圈选）")
        st.markdown("1️⃣ 在地图上绘制多边形\n2️⃣ **点击 Save 按钮**\n3️⃣ 填写信息并添加")
        
        if st.session_state.drawn_polygon:
            st.success(f"✅ 已捕获多边形，顶点数: {len(st.session_state.drawn_polygon)}")
        else:
            st.info("⏳ 尚未捕获多边形，请先绘制")
        
        new_obs_name = st.text_input("障碍物名称", placeholder="例如：新建筑")
        new_obs_height = st.number_input("高度 (米)", min_value=0, max_value=200, value=30)
        
        if st.button("✅ 添加已圈选的多边形"):
            if st.session_state.drawn_polygon and len(st.session_state.drawn_polygon) >= 3:
                if new_obs_name:
                    st.session_state.obstacles.append({
                        "name": new_obs_name,
                        "coords": st.session_state.drawn_polygon,
                        "height": new_obs_height
                    })
                    st.success(f"已添加障碍物: {new_obs_name}")
                    st.session_state.drawn_polygon = []
                    st.rerun()
                else:
                    st.error("请输入障碍物名称")
            else:
                st.error("请先在地图上绘制一个多边形（至少3个顶点）")

    # 坐标转换
    if is_gcj02:
        lat_a_display, lon_a_display = lat_a_input, lon_a_input
        lat_b_display, lon_b_display = lat_b_input, lon_b_input
    else:
        lon_a_gcj, lat_a_gcj = wgs84_to_gcj02(lon_a_input, lat_a_input)
        lon_b_gcj, lat_b_gcj = wgs84_to_gcj02(lon_b_input, lat_b_input)
        lat_a_display, lon_a_display = lat_a_gcj, lon_a_gcj
        lat_b_display, lon_b_display = lat_b_gcj, lon_b_gcj

    st.session_state.coords_a = {"lat": lat_a_display, "lon": lon_a_display}
    st.session_state.coords_b = {"lat": lat_b_display, "lon": lon_b_display}

    start = (lon_a_display, lat_a_display)
    end = (lon_b_display, lat_b_display)
    waypoints = calculate_avoidance_waypoints(
        start, end, st.session_state.obstacles, flight_height, safe_radius, strategy, bypass_offset
    )

    m_complete = create_complete_map(
        lat_a_display, lon_a_display, lat_b_display, lon_b_display,
        st.session_state.obstacles, flight_height, safe_radius, waypoints
    )
    output = st_folium(m_complete, width=900, height=600, key="map_complete")

    if output and output.get("last_active_drawing"):
        geo = output["last_active_drawing"].get("geometry", {})
        if geo.get("type") == "Polygon":
            coords = geo.get("coordinates", [])
            if coords:
                st.session_state.drawn_polygon = coords[0][:-1]

# ==================== 飞行监控页面 ====================
elif st.session_state.page == "飞行监控":
    st.title("📡 飞行监控 - 心跳监测")
    
    with st.sidebar:
        st.divider()
        st.header("🎮 心跳控制")
        col1, col2 = st.columns(2)
        with col1:
            if st.button("▶️ 开始模拟"):
                st.session_state.running = True
        with col2:
            if st.button("⏹️ 停止模拟"):
                st.session_state.running = False
        if st.button("🗑️ 清空数据"):
            st.session_state.heartbeats = []
            st.session_state.last_time = time.time()
            st.session_state.running = False
        st.divider()
        st.subheader("✈️ 当前航线")
        st.caption(f"起点A: {st.session_state.coords_a['lat']:.6f}, {st.session_state.coords_a['lon']:.6f}")
        st.caption(f"终点B: {st.session_state.coords_b['lat']:.6f}, {st.session_state.coords_b['lon']:.6f}")
        st.caption(f"高度: {st.session_state.flight_height} m | 安全半径: {st.session_state.safe_radius} m")
    
    def generate_heartbeat():
        seq = len(st.session_state.heartbeats) + 1
        now = datetime.now()
        st.session_state.heartbeats.append({
            "序号": seq,
            "时间": now,
            "延迟(秒)": round(time.time() - st.session_state.last_time, 3)
        })
        st.session_state.last_time = time.time()
    
    if st.session_state.running:
        if time.time() - st.session_state.last_time >= 1:
            generate_heartbeat()
            st.rerun()
    
    st.subheader("📊 实时状态")
    col1, col2, col3, col4 = st.columns(4)
    if st.session_state.heartbeats:
        latest = st.session_state.heartbeats[-1]
        seconds_since = time.time() - latest["时间"].timestamp()
        with col1:
            st.metric("最新序号", latest["序号"])
        with col2:
            st.metric("最后间隔", f"{latest['延迟(秒)']} 秒")
        with col3:
            st.metric("状态", "⚠️ 掉线" if seconds_since > 3 else "✅ 在线")
        with col4:
            st.metric("总心跳数", len(st.session_state.heartbeats))
        if seconds_since > 3:
            st.error(f"掉线！已 {seconds_since:.1f} 秒无心跳")
        else:
            st.success(f"在线 | 最后心跳: {latest['时间'].strftime('%H:%M:%S')}")
    else:
        col1.metric("---", "等待启动")
        col2.metric("---", "等待启动")
        col3.metric("---", "等待启动")
        col4.metric("---", "等待启动")
        st.info("点击「开始模拟」")
    
    st.divider()
    col1, col2 = st.columns([2,1])
    with col1:
        st.subheader("📈 心跳趋势")
        df = pd.DataFrame(st.session_state.heartbeats)
        if not df.empty:
            st.line_chart(df.set_index("时间")["序号"])
        else:
            st.info("暂无数据")
    with col2:
        st.subheader("📋 最近记录")
        if 'df' in locals() and not df.empty:
            st.dataframe(df.tail(10))
        else:
            st.info("暂无")
