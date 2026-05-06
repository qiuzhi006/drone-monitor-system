import streamlit as st
import pandas as pd
import time
import folium
from streamlit_folium import st_folium
import math
import json
import os
from datetime import datetime, timedelta

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
    # 1. 筛选威胁障碍物
    threatening = []
    for obs in obstacles:
        if obs['height'] >= flight_height:
            coords = obs['coords']
            center_lng = sum(c[0] for c in coords) / len(coords)
            center_lat = sum(c[1] for c in coords) / len(coords)
            # 计算最大半径
            max_r = max(math.hypot(c[0]-center_lng, c[1]-center_lat) for c in coords)
            threatening.append({
                'center': (center_lng, center_lat),
                'radius': max_r + safe_radius,  # 膨胀半径
                'coords': coords,
                'height': obs['height']
            })

    if strategy == 'direct' or not threatening:
        return [start, end]

    # 2. 简单检测：点是否在膨胀圆内
    def point_in_danger_zone(px, py):
        for obs in threatening:
            dist = math.hypot(px - obs['center'][0], py - obs['center'][1])
            if dist < obs['radius']:
                return True
        return False

    # 3. 生成绕行点：基于障碍物中心，在上方或下方生成明显的绕行路径
    waypoints = []
    
    # 找出所有有威胁的障碍物
    # 按距离起点排序
    threatening.sort(key=lambda obs: math.hypot(obs['center'][0]-start[0], obs['center'][1]-start[1]))
    
    # 对每个障碍物，生成3个固定绕行点
    all_detour_points = []
    for obs in threatening:
        center = obs['center']
        # 绕行距离：障碍物半径 + 额外安全余量
        detour_distance = obs['radius'] + bypass_offset * 2
        
        # 计算垂直于航线的方向
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length == 0:
            continue
        
        # 垂直向量（逆时针90度）
        perp_x = -dy / length
        perp_y = dx / length
        
        # 决定方向
        if strategy == 'left':
            direction_mult = 1
        elif strategy == 'right':
            direction_mult = -1
        else:
            # 默认向左（上方）
            direction_mult = 1
        
        # 生成3个绕行点，形成三角绕行路径
        detour_lat_rad = math.radians(center[1])
        meters_per_deg_lat = 111320.0
        meters_per_deg_lng = 111320.0 * math.cos(detour_lat_rad)
        
        # 绕行点1：障碍物前方
        lng1 = center[0] + (perp_x * detour_distance * direction_mult) / meters_per_deg_lng
        lat1 = center[1] + (perp_y * detour_distance * direction_mult) / meters_per_deg_lat
        
        # 绕行点2：障碍物正上方/下方
        lng2 = center[0] + (perp_x * detour_distance * 1.5 * direction_mult) / meters_per_deg_lng
        lat2 = center[1] + (perp_y * detour_distance * 1.5 * direction_mult) / meters_per_deg_lat
        
        # 绕行点3：障碍物后方
        lng3 = center[0] + (perp_x * detour_distance * direction_mult) / meters_per_deg_lng
        lat3 = center[1] + (perp_y * detour_distance * direction_mult) / meters_per_deg_lat
        
        all_detour_points.append((lng1, lat1))
        all_detour_points.append((lng2, lat2))
        all_detour_points.append((lng3, lat3))
    
    # 4. 构建最终路径：起点 -> 绕行点（按离起点距离排序） -> 终点
    if all_detour_points:
        # 按离起点的距离排序绕行点
        all_detour_points.sort(key=lambda p: math.hypot(p[0]-start[0], p[1]-start[1]))
        
        waypoints = [start] + all_detour_points + [end]
    else:
        waypoints = [start, end]
    
    return waypoints
# ==================== 初始化 Session State ====================
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

# 飞行模拟相关
if "flight_sim_running" not in st.session_state:
    st.session_state.flight_sim_running = False
if "flight_sim_start_time" not in st.session_state:
    st.session_state.flight_sim_start_time = None
if "flight_sim_current_index" not in st.session_state:
    st.session_state.flight_sim_current_index = 0
if "flight_sim_speed" not in st.session_state:
    st.session_state.flight_sim_speed = 8.5
if "flight_sim_waypoints" not in st.session_state:
    st.session_state.flight_sim_waypoints = []
if "flight_sim_total_distance" not in st.session_state:
    st.session_state.flight_sim_total_distance = 0
if "flight_sim_segment_distances" not in st.session_state:
    st.session_state.flight_sim_segment_distances = []

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
    st.title("📡 飞行实时画面 - 任务执行监控")
    
    # 计算总距离和各航段距离
    def calculate_distances(waypoints):
        total = 0
        segment_distances = []
        for i in range(len(waypoints) - 1):
            p1 = waypoints[i]
            p2 = waypoints[i + 1]
            lat1_rad = math.radians(p1[1])
            lat2_rad = math.radians(p2[1])
            dlat = math.radians(p2[1] - p1[1])
            dlng = math.radians(p2[0] - p1[0])
            a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlng/2)**2
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
            distance = 6371000 * c
            segment_distances.append(distance)
            total += distance
        return total, segment_distances
    
    # 侧边栏控制
    with st.sidebar:
        st.divider()
        st.header("🎮 飞行控制")
        
        if st.button("📐 导入当前航线", use_container_width=True):
            start = (st.session_state.coords_a["lon"], st.session_state.coords_a["lat"])
            end = (st.session_state.coords_b["lon"], st.session_state.coords_b["lat"])
            waypoints = calculate_avoidance_waypoints(
                start, end, st.session_state.obstacles,
                st.session_state.flight_height, st.session_state.safe_radius,
                st.session_state.avoidance_strategy, st.session_state.bypass_offset
            )
            total_dist, seg_dists = calculate_distances(waypoints)
            st.session_state.flight_sim_waypoints = waypoints
            st.session_state.flight_sim_total_distance = total_dist
            st.session_state.flight_sim_segment_distances = seg_dists
            st.session_state.flight_sim_current_index = 0
            st.session_state.flight_sim_running = False
            st.session_state.flight_sim_start_time = None
            st.success(f"航线已导入，共 {len(waypoints)} 个航点，总距离 {total_dist:.1f} 米")
        else:
            total_dist = st.session_state.flight_sim_total_distance
            waypoints = st.session_state.flight_sim_waypoints
            seg_dists = st.session_state.flight_sim_segment_distances
        
        st.divider()
        
        speed = st.slider("飞行速度 (m/s)", 1.0, 20.0, st.session_state.flight_sim_speed, 0.5)
        st.session_state.flight_sim_speed = speed
        
        st.divider()
        
        col1, col2 = st.columns(2)
        with col1:
            if st.button("▶️ 开始任务", use_container_width=True, disabled=len(waypoints) == 0):
                st.session_state.flight_sim_running = True
                if st.session_state.flight_sim_start_time is None:
                    st.session_state.flight_sim_start_time = time.time()
                st.rerun()
        with col2:
            if st.button("⏹️ 停止任务", use_container_width=True):
                st.session_state.flight_sim_running = False
                st.rerun()
        
        if st.button("🔄 重置任务", use_container_width=True):
            st.session_state.flight_sim_running = False
            st.session_state.flight_sim_start_time = None
            st.session_state.flight_sim_current_index = 0
            st.rerun()
        
        st.divider()
        st.subheader("📋 航线信息")
        st.caption(f"起点A: {st.session_state.coords_a['lat']:.6f}, {st.session_state.coords_a['lon']:.6f}")
        st.caption(f"终点B: {st.session_state.coords_b['lat']:.6f}, {st.session_state.coords_b['lon']:.6f}")
        st.caption(f"飞行高度: {st.session_state.flight_height} m")
        st.caption(f"安全半径: {st.session_state.safe_radius} m")
        st.caption(f"航点数量: {len(waypoints)}")
        if total_dist > 0:
            st.caption(f"总距离: {total_dist:.1f} 米")
    
    # 主界面
    if len(waypoints) == 0:
        st.warning("⚠️ 请先在侧边栏点击「📐 导入当前航线」按钮，加载航线规划结果")
    else:
        # 计算当前位置
        if st.session_state.flight_sim_running:
            elapsed_time = time.time() - st.session_state.flight_sim_start_time
            current_speed = st.session_state.flight_sim_speed
            flown_distance = elapsed_time * current_speed
            
            total_flown = 0
            current_index = 0
            segment_progress = 0
            
            for i, seg_dist in enumerate(seg_dists):
                if total_flown + seg_dist >= flown_distance:
                    current_index = i
                    if seg_dist > 0:
                        segment_progress = (flown_distance - total_flown) / seg_dist
                    break
                total_flown += seg_dist
            else:
                current_index = len(waypoints) - 1
                segment_progress = 1
                st.session_state.flight_sim_running = False
            
            st.session_state.flight_sim_current_index = current_index
            
            p1 = waypoints[current_index]
            p2_index = min(current_index + 1, len(waypoints) - 1)
            p2 = waypoints[p2_index]
            current_lng = p1[0] + (p2[0] - p1[0]) * segment_progress
            current_lat = p1[1] + (p2[1] - p1[1]) * segment_progress
            
            remaining_distance = max(0, total_dist - flown_distance)
            remaining_time = remaining_distance / current_speed if current_speed > 0 else 9999
            
            total_battery_time = 1800
            battery_remaining = max(0, 100 * (1 - min(elapsed_time, total_battery_time) / total_battery_time))
            
            hours = int(elapsed_time // 3600)
            minutes = int((elapsed_time % 3600) // 60)
            seconds = int(elapsed_time % 60)
            elapsed_str = f"{hours:02d}:{minutes:02d}:{seconds:02d}" if hours > 0 else f"{minutes:02d}:{seconds:02d}"
            
            if remaining_time >= 3600:
                rem_hours = int(remaining_time // 3600)
                rem_minutes = int((remaining_time % 3600) // 60)
                rem_seconds = int(remaining_time % 60)
                remaining_str = f"{rem_hours:02d}:{rem_minutes:02d}:{rem_seconds:02d}"
            elif remaining_time >= 0:
                rem_minutes = int(remaining_time // 60)
                rem_seconds = int(remaining_time % 60)
                remaining_str = f"{rem_minutes:02d}:{rem_seconds:02d}"
            else:
                remaining_str = "00:00"
            
            arrival_time = datetime.now() + timedelta(seconds=remaining_time)
            arrival_str = arrival_time.strftime("%H:%M:%S")
        else:
            current_lng = waypoints[0][0] if waypoints else st.session_state.coords_a["lon"]
            current_lat = waypoints[0][1] if waypoints else st.session_state.coords_a["lat"]
            flown_distance = 0
            remaining_distance = total_dist
            current_speed = 0
            elapsed_str = "00:00"
            remaining_str = "00:00"
            battery_remaining = 100
            arrival_str = "--:--:--"
            current_index = 0
        
        # 布局：左侧地图，右侧面板
        col_map, col_panel = st.columns([3, 1])
        
        with col_map:
            st.subheader("🗺️ 实时飞行地图")
            
            center_lat = (waypoints[0][1] + waypoints[-1][1]) / 2 if waypoints else 32.234
            center_lng = (waypoints[0][0] + waypoints[-1][0]) / 2 if waypoints else 118.751
            
            m = folium.Map(
                location=[center_lat, center_lng],
                zoom_start=17,
                tiles='https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
                attr='高德卫星地图',
                height=500
            )
            
            # 规划航线
            folium.PolyLine(
                locations=[(p[1], p[0]) for p in waypoints],
                color='gray',
                weight=3,
                opacity=0.6,
                dash_array='5,5',
                tooltip='规划航线'
            ).add_to(m)
            
            # 已飞行路径
            if st.session_state.flight_sim_running and flown_distance > 0:
                flown_waypoints = [waypoints[0]]
                total_check = 0
                for i, seg_dist in enumerate(seg_dists):
                    total_check += seg_dist
                    if total_check <= flown_distance:
                        flown_waypoints.append(waypoints[i + 1])
                    else:
                        flown_waypoints.append((current_lng, current_lat))
                        break
                if len(flown_waypoints) >= 2:
                    folium.PolyLine(
                        locations=[(p[1], p[0]) for p in flown_waypoints],
                        color='red',
                        weight=4,
                        opacity=0.9,
                        tooltip='已飞行路径'
                    ).add_to(m)
            
            # 航点标记
            for i, (lng, lat) in enumerate(waypoints):
                if i == 0:
                    color = 'green'
                    icon_name = 'play'
                elif i == len(waypoints) - 1:
                    color = 'red'
                    icon_name = 'flag-checkered'
                else:
                    color = 'blue'
                    icon_name = 'circle'
                folium.Marker(
                    location=[lat, lng],
                    popup=f'航点 {i+1}',
                    icon=folium.Icon(color=color, icon=icon_name, prefix='fa')
                ).add_to(m)
            
            # 障碍物
            for obs in st.session_state.obstacles:
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
            
            # 无人机当前位置
            folium.Marker(
                location=[current_lat, current_lng],
                popup='无人机当前位置',
                icon=folium.Icon(color='red', icon='plane', prefix='fa'),
                z_index_offset=1000
            ).add_to(m)
            
            # 安全半径圈
            if st.session_state.safe_radius > 0:
                folium.Circle(
                    location=[current_lat, current_lng],
                    radius=st.session_state.safe_radius,
                    color='red',
                    fill=True,
                    fill_opacity=0.1,
                    weight=1,
                    dash_array='5,5'
                ).add_to(m)
            
            st_folium(m, width=750, height=500, key="flight_monitor_map")
        
        with col_panel:
            st.subheader("📊 飞行数据")
            
            total_waypoints = len(waypoints)
            completed_waypoints = min(current_index + 1, total_waypoints) if st.session_state.flight_sim_running else 0
            st.metric("当前航点", f"{completed_waypoints}/{total_waypoints}")
            
            display_speed = current_speed if st.session_state.flight_sim_running else 0
            st.metric("飞行速度", f"{display_speed:.1f} m/s")
            
            st.metric("已用时间", elapsed_str)
            
            st.metric("剩余距离", f"{remaining_distance:.0f} m")
            
            st.metric("预计到达", remaining_str)
            
            st.metric("电量模拟", f"{battery_remaining:.0f}%")
            st.progress(int(battery_remaining) / 100)
            
            st.divider()
            
            st.subheader("🔗 通信链路")
            st.success("✅ GCS在线")
            st.success("✅ OBC在线")
            st.success("✅ FCU在线")
            
            st.divider()
            
            if st.session_state.flight_sim_running:
                st.info("✈️ 任务执行中...")
            elif current_index >= len(waypoints) - 1 and len(waypoints) > 0:
                st.success("✅ 任务已完成！")
            else:
                st.info("⏸️ 等待开始")
        
        # 自动刷新
        if st.session_state.flight_sim_running:
            time.sleep(1)
            st.rerun()
