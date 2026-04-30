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
    # 1. 筛选需要躲避的障碍物（高于飞行高度的）
    threatening = []
    for obs in obstacles:
        if obs['height'] >= flight_height:
            center_lng = sum(c[0] for c in obs['coords']) / len(obs['coords'])
            center_lat = sum(c[1] for c in obs['coords']) / len(obs['coords'])
            dist = point_to_segment_distance(center_lng, center_lat, start[0], start[1], end[0], end[1])
            max_r = max(math.hypot(c[0]-center_lng, c[1]-center_lat) for c in obs['coords'])
            # 使用安全半径来判断是否构成威胁
            if dist < safe_radius + max_r:
                threatening.append({
                    'center': (center_lng, center_lat),
                    'radius': max_r,
                    'height': obs['height']
                })

    # 无威胁或策略为直飞时，直接返回起终点
    if strategy == 'direct' or not threatening:
        return [start, end]

    waypoints = [start]
    current_start = start

    # 按离起点的距离排序，逐个处理
    threatening.sort(key=lambda x: point_to_segment_distance(x['center'][0], x['center'][1], start[0], start[1], end[0], end[1]))

    for obs in threatening:
        center = obs['center']
        # 总安全距离 = 障碍物半径 + 安全半径
        total_safe_dist = obs['radius'] + safe_radius

        # 使用 bypass_offset 作为最小绕行距离，但确保至少大于总安全距离
        bypass_dist = max(bypass_offset, total_safe_dist * 1.1)

        # 计算在原始航线上的投影点
        closest = get_closest_point_on_segment(center[0], center[1], current_start[0], current_start[1], end[0], end[1])

        # 决定绕行方向
        if strategy == 'left':
            direction = 'left'
        elif strategy == 'right':
            direction = 'right'
        else:
            # 最佳策略：选离终点更近的一侧
            left_pt = perpendicular_point(closest[0], closest[1], current_start[0], current_start[1], end[0], end[1], bypass_dist, 'left')
            right_pt = perpendicular_point(closest[0], closest[1], current_start[0], current_start[1], end[0], end[1], bypass_dist, 'right')
            dist_left = math.hypot(left_pt[0]-end[0], left_pt[1]-end[1])
            dist_right = math.hypot(right_pt[0]-end[0], right_pt[1]-end[1])
            direction = 'left' if dist_left < dist_right else 'right'

        # --- 关键改进：生成多个绕行航点，沿障碍物边缘平滑过渡 ---
        num_arc_points = 8  # 可调整，点数越多越平滑
        arc_waypoints = []

        # 计算从投影点指向障碍物中心的方向向量
        dx_to_center = center[0] - closest[0]
        dy_to_center = center[1] - closest[1]
        dist_to_center = math.hypot(dx_to_center, dy_to_center)

        if dist_to_center > 0:
            # 单位向量指向中心
            ux_center = dx_to_center / dist_to_center
            uy_center = dy_to_center / dist_to_center

            # 根据绕行方向决定扫描角度的方向
            # 顺时针扫描（右侧绕行）或逆时针扫描（左侧绕行）
            scan_direction = 1 if direction == 'right' else -1
            
            # 基础角度：从中心指向投影点的方向
            base_angle = math.atan2(-uy_center, -ux_center)  # 反向（从中心指向外）
            
            # 半圆扫描角度范围
            start_angle = base_angle - (scan_direction * math.pi / 2)
            end_angle = base_angle + (scan_direction * math.pi / 2)

            for i in range(num_arc_points + 1):
                # 当前角度（在外半圆上均匀分布）
                t = i / num_arc_points
                angle = start_angle + t * (end_angle - start_angle)
                
                # 计算绕行点：中心 + 绕行距离 * 方向向量
                arc_lng = center[0] + bypass_dist * math.cos(angle)
                arc_lat = center[1] + bypass_dist * math.sin(angle)
                arc_waypoints.append((arc_lng, arc_lat))

            # 根据绕行方向，决定航点顺序（保证路径连贯）
            if direction == 'right':
                arc_waypoints.reverse()
            
            waypoints.extend(arc_waypoints)
        else:
            # 投影点与中心重合的极端情况，退化为简单偏移
            waypoint = perpendicular_point(closest[0], closest[1], current_start[0], current_start[1], end[0], end[1], bypass_dist, direction)
            waypoints.append(waypoint)

        # 更新当前起点为最后一个绕行点
        current_start = waypoints[-1]

    waypoints.append(end)
    return waypoints

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
