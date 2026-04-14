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

# 默认坐标（南京某地）
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
    st.session_state.obstacles = []
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
    
    # 高德卫星图
    m = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=17,
        tiles='https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德卫星地图',
        zoom_control=True
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
        # 确保坐标格式为 [[lat, lon], ...]
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
        
        # 高度标签（放在中心）
        if len(obs["coords"]) > 0:
            center_lng = sum(c[0] for c in obs["coords"]) / len(obs["coords"])
            center_lat = sum(c[1] for c in obs["coords"]) / len(obs["coords"])
            folium.Marker(
                location=[center_lat, center_lng],
                icon=folium.DivIcon(
                    html=f'<div style="font-size: 12px; font-weight: bold; color: #ff6600; background: rgba(255,255,255,0.8); padding: 2px 5px; border-radius: 4px;">{obs["height"]}m</div>'
                )
            ).add_to(m)
    
    # 飞行高度指示
    folium.Marker(
        location=[center_lat, center_lon],
        icon=folium.DivIcon(
            html=f'<div style="font-size: 14px; font-weight: bold; background: white; padding: 2px 6px; border-radius: 15px; border: 1px solid red;">✈️ {height}m</div>'
        )
    ).add_to(m)
    
    # 添加绘图控件
    draw = folium.plugins.Draw(
        draw_options={
            'polyline': False,
            'rectangle': False,
            'circle': False,
            'marker': False,
            'circlemarker': False,
            'polygon': True
        },
        edit_options={'edit': True, 'remove': True}
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
        lat_a_input = st.number_input("纬度 A", value=float(st.session_state.coords_a["lat"]), format="%.6f", key="in_lat_a")
        lon_a_input = st.number_input("经度 A", value=float(st.session_state.coords_a["lon"]), format="%.6f", key="in_lon_a")
        
        st.header("📍 终点 B")
        lat_b_input = st.number_input("纬度 B", value=float(st.session_state.coords_b["lat"]), format="%.6f", key="in_lat_b")
        lon_b_input = st.number_input("经度 B", value=float(st.session_state.coords_b["lon"]), format="%.6f", key="in_lon_b")
        
        st.header("✈️ 飞行参数")
        flight_height = st.slider("飞行高度 (m)", 20, 150, int(st.session_state.flight_height))
        st.session_state.flight_height = flight_height
        
        st.divider()
        st.subheader("🗂️ 障碍物管理")
        col1, col2 = st.columns(2)
        with col1:
            if st.button("💾 保存配置", use_container_width=True):
                save_obstacles()
        with col2:
            if st.button("📂 加载配置", use_container_width=True):
                load_obstacles()
                
        if st.button("🗑️ 清除全部", use_container_width=True):
            st.session_state.obstacles = []
            st.session_state.drawn_polygon = None
            st.rerun()
        
        st.divider()
        st.subheader("➕ 添加障碍物")
        st.markdown("1. 在地图左侧工具栏选择多边形工具 **⏣**")
        st.markdown("2. 绘制区域，**点击 Save 按钮**")
        st.markdown("3. 下方显示捕获后，填写信息并添加")
        
        # 状态显示
        if st.session_state.drawn_polygon:
            st.success(f"✅ 已捕获多边形 ({len(st.session_state.drawn_polygon)}个顶点)")
        else:
            st.info("⏳ 等待绘制...")
            
        new_obs_name = st.text_input("名称", placeholder="例如：高压塔")
        new_obs_height = st.number_input("高度 (米)", 0, 300, 30)
        
        if st.button("✅ 确认添加障碍物"):
            if st.session_state.drawn_polygon:
                if new_obs_name:
                    st.session_state.obstacles.append({
                        "name": new_obs_name,
                        "coords": st.session_state.drawn_polygon,
                        "height": new_obs_height
                    })
                    st.success("添加成功！")
                    st.session_state.drawn_polygon = None # 清空临时缓存
                    st.rerun()
                else:
                    st.warning("请输入名称")
            else:
                st.error("请先在地图上绘制并保存多边形")

    # --- 坐标处理逻辑 ---
    # 如果用户输入的是 WGS84，转换为 GCJ02 用于显示
    if is_gcj02:
        lat_a_disp, lon_a_disp = lat_a_input, lon_a_input
        lat_b_disp, lon_b_disp = lat_b_input, lon_b_input
    else:
        lon_a_disp, lat_a_disp = wgs84_to_gcj02(lon_a_input, lat_a_input)
        lon_b_disp, lat_b_disp = wgs84_to_gcj02(lon_b_input, lat_b_input)

    # 更新 Session State 用于地图渲染
    st.session_state.coords_a = {"lat": lat_a_disp, "lon": lon_a_disp}
    st.session_state.coords_b = {"lat": lat_b_disp, "lon": lon_b_disp}

    # 渲染地图
    m = create_map(
        st.session_state.coords_a["lat"], st.session_state.coords_a["lon"],
        st.session_state.coords_b["lat"], st.session_state.coords_b["lon"],
        st.session_state.obstacles,
        flight_height
    )
    
    st.subheader("🗺️ 操作地图")
    # 关键修复：增加 return_on_hover=False (默认) 即可，重点在于下面的解析逻辑
    output = st_folium(m, width=900, height=600, key="map_draw")

    # --- 修复后的数据捕获逻辑 ---
    if output:
        # 1. 优先检查 Save 动作 (last_active_draw)
        if output.get("last_active_draw"):
            draw_data = output["last_active_draw"]
            geometry = draw_data.get("geometry")
            
            if geometry and geometry["type"] == "Polygon":
                # 提取坐标 [[lng, lat], ...]
                coords_raw = geometry["coordinates"][0]
                # 确保闭合（去掉最后一个重复点，如果有的话，虽然folium通常处理得很好）
                polygon_coords = [[c[0], c[1]] for c in coords_raw]
                
                # 只有当新数据与旧数据不同时才更新，防止死循环
                if polygon_coords != st.session_state.drawn_polygon:
                    st.session_state.drawn_polygon = polygon_coords
                    st.rerun()

        # 2. 备选检查：有时候 Save 没反应，但 all_drawings 里有最新数据
        # 注意：这可能会导致轻微的性能消耗，但在单机应用中可接受
        elif output.get("all_drawings"):
            drawings = output["all_drawings"]
            if len(drawings) > 0:
                # 取最后一个绘制的图形
                last_draw = drawings[-1]
                if last_draw.get("geometry", {}).get("type") == "Polygon":
                    coords_raw = last_draw["geometry"]["coordinates"][0]
                    polygon_coords = [[c[0], c[1]] for c in coords_raw]
                    
                    if polygon_coords != st.session_state.drawn_polygon:
                         # 这里不自动 rerun，避免用户还在画的时候频繁刷新，
                         # 但可以在界面上提示用户“检测到新图形，请点击 Save”
                         pass

    # 调试用：显示原始数据（可折叠）
    with st.expander("🐞 调试：查看地图返回数据"):
        st.json(output)

    # 底部图例
    st.markdown("---")
    c1, c2, c3 = st.columns(3)
    c1.markdown("🟢 **起点**")
    c2.markdown("🔴 **终点**")
    c3.markdown("🟠 **障碍物**")

# ==================== 飞行监控页面 ====================
else:
    st.title("📡 飞行监控 - 心跳监测")
    
    with st.sidebar:
        st.header("🎮 控制")
        col1, col2 = st.columns(2)
        with col1:
            if st.button("▶️ 开始", use_container_width=True):
                st.session_state.running = True
        with col2:
            if st.button("⏹️ 停止", use_container_width=True):
                st.session_state.running = False
                
        if st.button("🗑️ 清空记录", use_container_width=True):
            st.session_state.heartbeats = []
            st.rerun()
            
        st.divider()
        st.info(f"**当前任务**\nA: {st.session_state.coords_a['lat']:.4f}, {st.session_state.coords_a['lon']:.4f}\nB: {st.session_state.coords_b['lat']:.4f}, {st.session_state.coords_b['lon']:.4f}")

    # 模拟心跳
    if st.session_state.running:
        current_time = time.time()
        if current_time - st.session_state.last_time >= 1.0: # 1秒一次
            seq = len(st.session_state.heartbeats) + 1
            st.session_state.heartbeats.append({
                "序号": seq,
                "时间": datetime.now().strftime("%H:%M:%S"),
                "延迟": round(time.time() - st.session_state.last_time, 3),
                "状态": "正常"
            })
            st.session_state.last_time = current_time
            st.rerun()

    # 仪表盘
    st.subheader("📊 实时状态")
    if st.session_state.heartbeats:
        latest = st.session_state.heartbeats[-1]
        # 简单的掉线判断逻辑
        is_online = (time.time() - st.session_state.last_time) < 3.0
        
        c1, c2, c3 = st.columns(3)
        c1.metric("最新序号", latest["序号"])
        c2.metric("信号延迟", f"{latest['延迟']}s", delta_color="inverse")
        c3.metric("连接状态", "🟢 在线" if is_online else "🔴 掉线", delta="实时更新")
        
        if not is_online:
            st.error("⚠️ 警告：超过3秒未收到心跳包！")
    else:
        st.info("等待启动模拟...")

    # 图表
    if st.session_state.heartbeats:
        df = pd.DataFrame(st.session_state.heartbeats)
        st.line_chart(df.set_index("时间")["序号"])
        st.dataframe(df, use_container_width=True)
