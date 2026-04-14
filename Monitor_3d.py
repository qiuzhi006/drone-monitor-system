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
if "coord_system" not in st.session_state:
    st.session_state.coord_system = "GCJ-02 (高德/腾讯)"
if "page" not in st.session_state:
    st.session_state.page = "飞行监控"
if "obstacles" not in st.session_state:
    st.session_state.obstacles = []   # 改为空列表，让用户自己添加
if "drawn_polygon" not in st.session_state:
    st.session_state.drawn_polygon = None

# 持久化文件路径
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
            st.error(f"加载配置文件失败: {e}")

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
def create_map(lat_a, lon_a, lat_b, lon_b, obstacles, height):
    center_lat = (lat_a + lat_b) / 2
    center_lon = (lon_a + lon_b) / 2
    
    # 高德卫星图（style=6 卫星图，支持GCJ-02）
    m = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=17,
        tiles='https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德卫星地图'
    )
    
    # 航线
    folium.PolyLine(
        locations=[[lat_a, lon_a], [lat_b, lon_b]],
        color='red',
        weight=5,
        opacity=0.8,
        tooltip='飞行航线'
    ).add_to(m)
    
    # 起点
    folium.Marker(
        location=[lat_a, lon_a],
        popup=f'起点A<br>纬度: {lat_a:.6f}<br>经度: {lon_a:.6f}',
        icon=folium.Icon(color='green', icon='play', prefix='fa')
    ).add_to(m)
    
    # 终点
    folium.Marker(
        location=[lat_b, lon_b],
        popup=f'终点B<br>纬度: {lat_b:.6f}<br>经度: {lon_b:.6f}',
        icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')
    ).add_to(m)
    
    # 障碍物多边形
    for obs in obstacles:
        polygon_coords = [[coord[1], coord[0]] for coord in obs["coords"]]  # [lat, lng]
        folium.Polygon(
            locations=polygon_coords,
            color='orange',
            fill=True,
            fill_color='orange',
            fill_opacity=0.4,
            weight=2,
            tooltip=f"{obs['name']} (高{obs['height']}m)"
        ).add_to(m)
        # 高度标签
        center = [sum(c[1] for c in obs["coords"])/len(obs["coords"]),
                  sum(c[0] for c in obs["coords"])/len(obs["coords"])]
        folium.Marker(
            location=[center[0], center[1]],
            icon=folium.DivIcon(
                html=f'<div style="font-size: 12px; font-weight: bold; color: #ff6600;">{obs["height"]}m</div>'
            )
        ).add_to(m)
    
    # 飞行高度指示
    folium.Marker(
        location=[center_lat, center_lon],
        icon=folium.DivIcon(
            html=f'<div style="font-size: 14px; font-weight: bold; background: white; padding: 2px 6px; border-radius: 15px; border: 1px solid red;">✈️ 飞行高度: {height}米</div>'
        )
    ).add_to(m)
    
    # 添加绘图控件（只允许绘制多边形）
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
            index=0 if st.session_state.coord_system == "GCJ-02 (高德/腾讯)" else 1
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
        
        st.header("✈️ 飞行参数")
        flight_height = st.slider("飞行高度 (m)", 20, 100, st.session_state.flight_height)
        st.session_state.flight_height = flight_height
        
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
            st.session_state.drawn_polygon = None
            st.success("已清除所有障碍物")
        
        st.divider()
        st.subheader("➕ 添加障碍物（多边形圈选）")
        st.markdown("1️⃣ 在地图上绘制多边形（工具栏⏢，最后**双击**结束）\n2️⃣ 捕获成功后下方会显示顶点数\n3️⃣ 填写名称和高度，点击添加")
        
        # 显示当前捕获的多边形状态
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
                    st.session_state.drawn_polygon = None
                    st.rerun()
                else:
                    st.error("请输入障碍物名称")
            else:
                st.error("请先在地图上绘制一个多边形（至少3个顶点）")
        
        # 备用方案：手动输入多边形
        st.divider()
        with st.expander("✏️ 手动输入多边形（备用）"):
            st.markdown("如果绘制工具无法捕获，可手动输入顶点坐标（经度,纬度），每行一个点。")
            st.code("示例：\n118.7488,32.2320\n118.7492,32.2320\n118.7492,32.2324\n118.7488,32.2324")
            manual_coords = st.text_area("多边形顶点（每行一对经纬度）", height=150)
            manual_name = st.text_input("障碍物名称（手动）", key="manual_name")
            manual_height = st.number_input("高度 (米)", 0, 200, 30, key="manual_height")
            if st.button("➕ 手动添加障碍物"):
                if manual_name and manual_coords.strip():
                    try:
                        coords = []
                        for line in manual_coords.strip().split('\n'):
                            lng, lat = map(float, line.strip().split(','))
                            coords.append([lng, lat])
                        if len(coords) >= 3:
                            st.session_state.obstacles.append({
                                "name": manual_name,
                                "coords": coords,
                                "height": manual_height
                            })
                            st.success(f"已添加障碍物: {manual_name}")
                            st.rerun()
                        else:
                            st.error("至少需要3个顶点")
                    except Exception as e:
                        st.error(f"坐标格式错误: {e}")
                else:
                    st.error("请输入名称和顶点坐标")
        
        with st.expander("📋 当前障碍物列表"):
            if not st.session_state.obstacles:
                st.write("暂无障碍物")
            for i, obs in enumerate(st.session_state.obstacles):
                st.write(f"{i+1}. {obs['name']} (高{obs['height']}m) - {len(obs['coords'])}个顶点")
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
    
    st.subheader("🗺️ 高德卫星地图 - 绘制多边形圈选障碍物")
    
    # 创建地图
    m = create_map(
        lat_a_display, lon_a_display,
        lat_b_display, lon_b_display,
        st.session_state.obstacles,
        flight_height
    )
    
    # 使用 st_folium 并捕获绘图数据
    output = st_folium(m, width=900, height=600, key="map_draw")
    
    # 解析用户绘制的多边形（兼容不同版本的输出格式）
    if output and "last_active_draw" in output and output["last_active_draw"]:
        draw_data = output["last_active_draw"]
        # 尝试提取多边形坐标
        if "geometry" in draw_data and draw_data["geometry"]["type"] == "Polygon":
            coords_original = draw_data["geometry"]["coordinates"][0]
            # 转换为 [[lng, lat], ...]
            polygon_coords = [[c[0], c[1]] for c in coords_original]
            st.session_state.drawn_polygon = polygon_coords
            st.success(f"✅ 已捕获多边形（{len(polygon_coords)}个顶点），请填写名称和高度后点击添加")
        elif "geometry" in draw_data and draw_data["geometry"]["type"] == "MultiPolygon":
            # 取第一个多边形
            coords_original = draw_data["geometry"]["coordinates"][0][0]
            polygon_coords = [[c[0], c[1]] for c in coords_original]
            st.session_state.drawn_polygon = polygon_coords
            st.success(f"✅ 已捕获多边形（{len(polygon_coords)}个顶点），请填写名称和高度后点击添加")
    
    # 图例
    col1, col2, col3, col4 = st.columns(4)
    with col1:
        st.markdown("🟢 **绿色标记** = 起点A")
    with col2:
        st.markdown("🔴 **红色标记** = 终点B")
    with col3:
        st.markdown("🟠 **橙色多边形** = 障碍物")
    with col4:
        st.markdown("🔴 **红线** = 飞行航线")
    
    st.divider()
    st.subheader("📐 坐标信息")
    col1, col2 = st.columns(2)
    with col1:
        st.info(f"**起点A** (GCJ-02)\n- 纬度: {lat_a_display:.6f}\n- 经度: {lon_a_display:.6f}")
    with col2:
        st.info(f"**终点B** (GCJ-02)\n- 纬度: {lat_b_display:.6f}\n- 经度: {lon_b_display:.6f}")
    
    st.caption(f"飞行高度: {flight_height} 米 | 障碍物数量: {len(st.session_state.obstacles)} 个 | 坐标系: {coord_system}")

# ==================== 飞行监控页面 ====================
else:
    st.title("📡 飞行监控 - 心跳监测")
    
    with st.sidebar:
        st.divider()
        st.header("🎮 心跳控制")
        col1, col2 = st.columns(2)
        with col1:
            if st.button("▶️ 开始模拟", use_container_width=True):
                st.session_state.running = True
        with col2:
            if st.button("⏹️ 停止模拟", use_container_width=True):
                st.session_state.running = False
        if st.button("🗑️ 清空数据", use_container_width=True):
            st.session_state.heartbeats = []
            st.session_state.last_time = time.time()
            st.session_state.running = False
        st.divider()
        st.subheader("✈️ 当前航线")
        st.caption(f"起点A: {st.session_state.coords_a['lat']:.6f}, {st.session_state.coords_a['lon']:.6f}")
        st.caption(f"终点B: {st.session_state.coords_b['lat']:.6f}, {st.session_state.coords_b['lon']:.6f}")
        st.caption(f"飞行高度: {st.session_state.flight_height} 米")
        st.caption(f"坐标系: {st.session_state.coord_system}")
    
    # 心跳生成
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
        current_time = time.time()
        if current_time - st.session_state.last_time >= 1:
            generate_heartbeat()
            st.rerun()
    
    # 状态卡片
    st.subheader("📊 实时状态")
    col1, col2, col3, col4 = st.columns(4)
    if len(st.session_state.heartbeats) > 0:
        latest = st.session_state.heartbeats[-1]
        last_beat_time = latest["时间"].timestamp()
        seconds_since = time.time() - last_beat_time
        
        with col1:
            st.metric("最新心跳序号", latest["序号"])
        with col2:
            st.metric("最后心跳间隔", f"{latest['延迟(秒)']} 秒")
        with col3:
            if seconds_since > 3:
                st.metric("连接状态", "⚠️ 掉线", delta=f"{seconds_since:.1f}秒无响应")
            else:
                st.metric("连接状态", "✅ 在线", delta=f"{seconds_since:.1f}秒前")
        with col4:
            st.metric("总心跳数", len(st.session_state.heartbeats))
        
        if seconds_since > 3:
            st.error(f"🚨 无人机掉线！已 {seconds_since:.1f} 秒未收到心跳包！")
        else:
            st.success(f"📡 无人机在线 | 最后心跳: {latest['时间'].strftime('%H:%M:%S')}")
    else:
        for col in [col1, col2, col3, col4]:
            with col:
                st.metric("---", "等待启动")
        st.info("点击左侧「开始模拟」启动心跳监测")
    
    st.divider()
    
    # 可视化
    col1, col2 = st.columns([2, 1])
    with col1:
        st.subheader("📈 心跳序号变化趋势")
        df = pd.DataFrame(st.session_state.heartbeats)
        if not df.empty:
            st.line_chart(df.set_index("时间")["序号"], use_container_width=True)
        else:
            st.info("暂无心跳数据")
    with col2:
        st.subheader("📋 最近心跳记录")
        if not df.empty:
            st.dataframe(df.tail(10), use_container_width=True)
        else:
            st.info("暂无数据")
    st.caption("提示：每秒自动发送一次心跳包，超过3秒无响应触发掉线报警")
