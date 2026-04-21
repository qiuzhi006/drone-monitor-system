import streamlit as st
import pandas as pd
import time
import folium
from streamlit_folium import folium_static
import math
import json
import os
from datetime import datetime
from typing import List, Tuple

st.set_page_config(layout="wide", page_title="无人机监测系统")

# ==================== 坐标转换函数 ====================
pi = 3.1415926535897932384626
a = 6378245.0
ee = 0.00669342162296594323

def _transform_lat(lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + \
          0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 *
            math.sin(2.0 * lng * pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * pi) + 40.0 *
            math.sin(lat / 3.0 * pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * pi) + 320 *
            math.sin(lat * pi / 30.0)) * 2.0 / 3.0
    return ret

def _transform_lng(lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + \
          0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 *
            math.sin(2.0 * lng * pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * pi) + 40.0 *
            math.sin(lng / 3.0 * pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lng / 12.0 * pi) + 300.0 *
            math.sin(lng / 30.0 * pi)) * 2.0 / 3.0
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

# ==================== 几何辅助函数 ====================
def point_to_segment_distance(px, py, x1, y1, x2, y2):
    """计算点到线段的最短距离"""
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    if t < 0:
        t = 0
    elif t > 1:
        t = 1
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.hypot(px - proj_x, py - proj_y)

def get_closest_point_on_segment(px, py, x1, y1, x2, y2):
    """获取线段上离点最近的点坐标"""
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return x1, y1
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0, min(1, t))
    return x1 + t * dx, y1 + t * dy

def perpendicular_point(px, py, x1, y1, x2, y2, offset, direction='left'):
    """
    计算线段上某点的垂直偏移点
    direction: 'left' 或 'right'，相对于从(x1,y1)到(x2,y2)的方向
    """
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)
    if length == 0:
        return px + offset, py + offset
    # 单位方向向量
    ux = dx / length
    uy = dy / length
    # 垂直向量（顺时针旋转90度）
    perp_x = -uy
    perp_y = ux
    if direction == 'right':
        perp_x = uy
        perp_y = -ux
    return px + perp_x * offset, py + perp_y * offset

def calculate_avoidance_waypoints(start, end, obstacles, flight_height, safe_radius, strategy):
    """
    计算避障航线
    start, end: (lng, lat)
    obstacles: 障碍物列表，每个包含 'coords' (多边形顶点列表) 和 'height'
    flight_height: 无人机飞行高度
    safe_radius: 安全半径（米）
    strategy: 'direct', 'left', 'right', 'best'
    返回航线点列表（经纬度），如果strategy='direct'且高度足够则返回[start, end]
    """
    # 将高度不足的障碍物筛选出来
    threatening = []
    for obs in obstacles:
        if obs['height'] >= flight_height:  # 需要绕行
            # 计算多边形中心点（简化）
            center_lng = sum(c[0] for c in obs['coords']) / len(obs['coords'])
            center_lat = sum(c[1] for c in obs['coords']) / len(obs['coords'])
            # 计算中心点到航线的最短距离
            dist = point_to_segment_distance(center_lng, center_lat, start[0], start[1], end[0], end[1])
            # 计算障碍物半径（近似为多边形外接圆半径）
            max_r = max(math.hypot(c[0]-center_lng, c[1]-center_lat) for c in obs['coords'])
            if dist < safe_radius + max_r:
                threatening.append({
                    'center': (center_lng, center_lat),
                    'radius': max_r,
                    'height': obs['height']
                })
    if strategy == 'direct' or not threatening:
        return [start, end]
    
    # 需要绕行：生成绕行点
    # 对于每个威胁障碍物，在航线上找最近点，然后偏移生成绕行点
    waypoints = [start]
    current_start = start
    # 按距起点距离排序
    threatening.sort(key=lambda x: point_to_segment_distance(x['center'][0], x['center'][1], start[0], start[1], end[0], end[1]))
    for obs in threatening:
        center = obs['center']
        radius = obs['radius']
        # 找到当前线段上离中心最近的点
        closest = get_closest_point_on_segment(center[0], center[1], current_start[0], current_start[1], end[0], end[1])
        # 偏移距离 = safe_radius + radius
        offset_dist = safe_radius + radius
        # 确定偏移方向
        if strategy == 'left':
            direction = 'left'
        elif strategy == 'right':
            direction = 'right'
        else:  # best: 选择偏移后离终点更近的方向
            left_pt = perpendicular_point(closest[0], closest[1], current_start[0], current_start[1], end[0], end[1], offset_dist, 'left')
            right_pt = perpendicular_point(closest[0], closest[1], current_start[0], current_start[1], end[0], end[1], offset_dist, 'right')
            dist_left = math.hypot(left_pt[0]-end[0], left_pt[1]-end[1])
            dist_right = math.hypot(right_pt[0]-end[0], right_pt[1]-end[1])
            direction = 'left' if dist_left < dist_right else 'right'
        # 生成绕行点
        waypoint = perpendicular_point(closest[0], closest[1], current_start[0], current_start[1], end[0], end[1], offset_dist, direction)
        waypoints.append(waypoint)
        current_start = waypoint
    waypoints.append(end)
    return waypoints

# ==================== 初始化 Session State ====================
if "heartbeats" not in st.session_state:
    st.session_state.heartbeats = []
    st.session_state.last_time = time.time()
    st.session_state.running = False
if "coords_a" not in st.session_state:
    st.session_state.coords_a = {"lat": 32.2305, "lon": 118.7485}
if "coords_b" not in st.session_state:
    st.session_state.coords_b = {"lat": 32.2365, "lon": 118.7500}
if "flight_height" not in st.session_state:
    st.session_state.flight_height = 50
if "safe_radius" not in st.session_state:
    st.session_state.safe_radius = 5.0   # 默认5米
if "coord_system" not in st.session_state:
    st.session_state.coord_system = "GCJ-02 (高德/腾讯)"
if "page" not in st.session_state:
    st.session_state.page = "飞行监控"
if "obstacles" not in st.session_state:
    # 默认障碍物（南京科技职业学院）
    st.session_state.obstacles = [
        {
            "name": "教学楼1",
            "coords": [[118.7488, 32.2320], [118.7492, 32.2320], [118.7492, 32.2324], [118.7488, 32.2324]],
            "height": 30
        },
        {
            "name": "教学楼2",
            "coords": [[118.7490, 32.2332], [118.7494, 32.2332], [118.7494, 32.2336], [118.7490, 32.2336]],
            "height": 35
        },
        {
            "name": "图书馆",
            "coords": [[118.7492, 32.2340], [118.7496, 32.2340], [118.7496, 32.2344], [118.7492, 32.2344]],
            "height": 25
        },
        {
            "name": "食堂",
            "coords": [[118.7495, 32.2348], [118.7499, 32.2348], [118.7499, 32.2352], [118.7495, 32.2352]],
            "height": 20
        },
        {
            "name": "宿舍楼",
            "coords": [[118.7498, 32.2355], [118.7502, 32.2355], [118.7502, 32.2359], [118.7498, 32.2359]],
            "height": 28
        }
    ]
if "avoidance_strategy" not in st.session_state:
    st.session_state.avoidance_strategy = "best"

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

# ==================== 创建地图函数（显示航线） ====================
def create_map(lat_a, lon_a, lat_b, lon_b, obstacles, flight_height, safe_radius, strategy, waypoints=None):
    center_lat = (lat_a + lat_b) / 2
    center_lon = (lon_a + lon_b) / 2
    
    m = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=17,
        tiles='https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德卫星地图'
    )
    
    # 原始航线（灰色虚线）
    folium.PolyLine(
        locations=[[lat_a, lon_a], [lat_b, lon_b]],
        color='gray', weight=3, opacity=0.5, dash_array='5,5',
        tooltip='原始航线'
    ).add_to(m)
    
    # 避障航线（红色实线）
    if waypoints is None:
        waypoints = calculate_avoidance_waypoints(
            (lon_a, lat_a), (lon_b, lat_b), obstacles, flight_height, safe_radius, strategy
        )
    # 转换为folium坐标格式 (lat, lng)
    folium_points = [(p[1], p[0]) for p in waypoints]
    folium.PolyLine(
        locations=folium_points,
        color='red', weight=5, opacity=0.8,
        tooltip='规划航线'
    ).add_to(m)
    
    # 添加航线点标记
    for i, (lng, lat) in enumerate(waypoints):
        folium.CircleMarker(
            location=[lat, lng],
            radius=4,
            color='blue' if i==0 or i==len(waypoints)-1 else 'orange',
            fill=True,
            popup=f'航点{i}'
        ).add_to(m)
    
    # 起点和终点
    folium.Marker(
        location=[lat_a, lon_a],
        popup=f'起点A<br>{lat_a:.6f},{lon_a:.6f}',
        icon=folium.Icon(color='green', icon='play', prefix='fa')
    ).add_to(m)
    folium.Marker(
        location=[lat_b, lon_b],
        popup=f'终点B<br>{lat_b:.6f},{lon_b:.6f}',
        icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')
    ).add_to(m)
    
    # 障碍物多边形
    for obs in obstacles:
        polygon_coords = [[c[1], c[0]] for c in obs["coords"]]
        color = 'orange' if obs['height'] < flight_height else 'darkred'
        folium.Polygon(
            locations=polygon_coords,
            color=color, fill=True, fill_color=color, fill_opacity=0.4,
            weight=2, tooltip=f"{obs['name']} (高{obs['height']}m)"
        ).add_to(m)
        # 高度标签
        center_lat_obs = sum(c[1] for c in obs["coords"]) / len(obs["coords"])
        center_lon_obs = sum(c[0] for c in obs["coords"]) / len(obs["coords"])
        folium.Marker(
            location=[center_lat_obs, center_lon_obs],
            icon=folium.DivIcon(html=f'<div style="font-size:12px; font-weight:bold; color:{color};">{obs["height"]}m</div>')
        ).add_to(m)
    
    # 飞行参数说明
    folium.Marker(
        location=[center_lat, center_lon],
        icon=folium.DivIcon(html=f'<div style="font-size:14px; font-weight:bold; background:white; padding:2px 6px; border-radius:15px; border:1px solid red;">✈️ 高度:{flight_height}m | 安全半径:{safe_radius}m</div>')
    ).add_to(m)
    
    return m, waypoints

# ==================== 航线规划页面 ====================
if st.session_state.page == "航线规划":
    st.title("🗺️ 航线规划 + 避障策略")
    
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
            if st.button("💾 保存障碍物"):
                save_obstacles()
        with col2:
            if st.button("📂 加载障碍物"):
                load_obstacles()
        if st.button("🗑️ 清除全部障碍物"):
            st.session_state.obstacles = []
            st.success("已清除所有障碍物")
        
        st.divider()
        st.subheader("➕ 手动添加障碍物（多边形圈选）")
        st.markdown("从高德坐标拾取器获取顶点坐标（经度,纬度），每行一个点，至少3个点。")
        st.caption("示例：\n118.7488,32.2320\n118.7492,32.2320\n118.7492,32.2324\n118.7488,32.2324")
        manual_coords = st.text_area("多边形顶点（每行一对经纬度）", height=150, key="manual_coords")
        manual_name = st.text_input("障碍物名称", key="manual_name")
        manual_height = st.number_input("高度 (米)", 0, 200, 30, key="manual_height")
        if st.button("➕ 添加障碍物"):
            if not manual_name.strip():
                st.error("请输入名称")
            elif not manual_coords.strip():
                st.error("请输入顶点坐标")
            else:
                try:
                    coords = []
                    for line in manual_coords.strip().split('\n'):
                        line = line.strip()
                        if not line:
                            continue
                        lng, lat = map(float, line.split(','))
                        coords.append([lng, lat])
                    if len(coords) < 3:
                        st.error(f"至少需要3个顶点，当前{len(coords)}个")
                    else:
                        st.session_state.obstacles.append({
                            "name": manual_name.strip(),
                            "coords": coords,
                            "height": manual_height
                        })
                        st.success(f"已添加 {manual_name}")
                        st.rerun()
                except Exception as e:
                    st.error(f"坐标格式错误: {e}")
        
        with st.expander("📋 当前障碍物列表"):
            if not st.session_state.obstacles:
                st.write("暂无")
            for i, obs in enumerate(st.session_state.obstacles):
                st.write(f"{i+1}. {obs['name']} ({obs['height']}m) - {len(obs['coords'])}个顶点")
                if st.button(f"❌ 删除 {obs['name']}", key=f"del_{i}"):
                    st.session_state.obstacles.pop(i)
                    st.rerun()
    
    # 坐标转换
    if is_gcj02:
        lat_a_display, lon_a_display = lat_a_input, lon_a_input
        lat_b_display, lon_b_display = lat_b_input, lon_b_input
    else:
        lon_a_display, lat_a_display = wgs84_to_gcj02(lon_a_input, lat_a_input)
        lon_b_display, lat_b_display = wgs84_to_gcj02(lon_b_input, lat_b_input)
    
    st.session_state.coords_a = {"lat": lat_a_display, "lon": lon_a_display}
    st.session_state.coords_b = {"lat": lat_b_display, "lon": lon_b_display}
    
    # 计算航线点
    start = (lon_a_display, lat_a_display)
    end = (lon_b_display, lat_b_display)
    waypoints = calculate_avoidance_waypoints(
        start, end, st.session_state.obstacles,
        flight_height, safe_radius, strategy
    )
    
    st.subheader("🗺️ 高德卫星地图 - 航线规划")
    m, computed_waypoints = create_map(
        lat_a_display, lon_a_display,
        lat_b_display, lon_b_display,
        st.session_state.obstacles,
        flight_height, safe_radius, strategy,
        waypoints
    )
    folium_static(m, width=900, height=600)
    
    # 显示避障信息
    st.subheader("📋 航线规划报告")
    if strategy == 'direct':
        st.info("当前策略：直接飞。若飞行高度大于所有障碍物高度则无绕行；否则将按高度条件自动绕行。")
    else:
        st.success(f"已生成绕行航线，共 {len(waypoints)} 个航点。")
        # 显示航点坐标
        df_waypoints = pd.DataFrame(waypoints, columns=['经度', '纬度'])
        st.dataframe(df_waypoints)
    
    # 图例
    col1, col2, col3, col4 = st.columns(4)
    with col1:
        st.markdown("🟢 **绿色** = 起点A")
    with col2:
        st.markdown("🔴 **红色** = 终点B")
    with col3:
        st.markdown("🟠 **橙色多边形** = 障碍物(低于飞行高度)")
    with col4:
        st.markdown("🔴 **深红多边形** = 障碍物(高于飞行高度，需绕行)")
    st.caption("灰色虚线: 原始航线 | 红色实线: 规划航线 | 橙色/蓝色圆点: 航点")
    
    st.divider()
    st.subheader("📐 坐标信息")
    col1, col2 = st.columns(2)
    with col1:
        st.info(f"**起点A** (GCJ-02)\n- {lat_a_display:.6f}, {lon_a_display:.6f}")
    with col2:
        st.info(f"**终点B** (GCJ-02)\n- {lat_b_display:.6f}, {lon_b_display:.6f}")
    st.caption(f"飞行高度: {flight_height} 米 | 安全半径: {safe_radius} 米 | 障碍物数量: {len(st.session_state.obstacles)}")

# ==================== 飞行监控页面（保持不变） ====================
else:
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
        for _ in range(4):
            st.metric("---", "等待启动")
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
        if not df.empty:
            st.dataframe(df.tail(10))
        else:
            st.info("暂无")
