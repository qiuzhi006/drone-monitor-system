import streamlit as st
import pandas as pd
import time
import folium
from streamlit_folium import st_folium
from folium.plugins import Draw
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
    st.session_state.obstacles = []
if "drawn_polygon" not in st.session_state:
    st.session_state.drawn_polygon = None

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
        st.success("障碍物已保存")
    except Exception as e:
        st.error(f"保存失败: {e}")

# ==================== 侧边栏 ====================
with st.sidebar:
    st.title("🚁 导航")
    page = st.radio("功能页面", ["飞行监控", "航线规划"])
    st.session_state.page = page

# ==================== 创建地图 ====================
def create_map(lat_a, lon_a, lat_b, lon_b, obstacles, height):
    center_lat = (lat_a + lat_b) / 2
    center_lon = (lon_a + lon_b) / 2

    m = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=17,
        tiles='https://webst01.is.autavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
        attr='高德卫星地图'
    )

    # 航线
    folium.PolyLine(
        locations=[[lat_a, lon_a], [lat_b, lon_b]],
        color='red', weight=5, tooltip='飞行航线'
    ).add_to(m)

    # 起点/终点
    folium.Marker([lat_a, lon_a], popup='起点A',
                  icon=folium.Icon(color='green', icon='play')).add_to(m)
    folium.Marker([lat_b, lon_b], popup='终点B',
                  icon=folium.Icon(color='red', icon='flag')).add_to(m)

    # 障碍物
    for obs in obstacles:
        poly = [[c[1], c[0]] for c in obs["coords"]]
        folium.Polygon(
            locations=poly, color='orange', fill=True, fill_opacity=0.4,
            tooltip=f"{obs['name']} 高{obs['height']}m"
        ).add_to(m)

    # 绘制工具（只开多边形）
    draw = Draw(
        draw_options={
            'polyline': False, 'rectangle': False, 'circle': False,
            'marker': False, 'polygon': True
        },
        edit_options={'edit': True}
    )
    draw.add_to(m)
    return m

# ==================== 航线规划页面 ====================
if st.session_state.page == "航线规划":
    st.title("🗺️ 航线规划 + 障碍物圈选")

    with st.sidebar:
        st.subheader("📍 坐标设置")
        coord_system = st.selectbox(
            "坐标系", ["GCJ-02 (高德)", "WGS-84 (GPS)"]
        )
        is_gcj02 = "GCJ-02" in coord_system

        lat_a = st.number_input("A纬度", value=32.2305, format="%.6f")
        lon_a = st.number_input("A经度", value=118.7485, format="%.6f")
        lat_b = st.number_input("B纬度", value=32.2365, format="%.6f")
        lon_b = st.number_input("B经度", value=118.7500, format="%.6f")
        height = st.slider("飞行高度(m)", 20, 100, 50)

        # 保存/加载/清除
        col1, col2 = st.columns(2)
        with col1:
            if st.button("💾 保存"):
                save_obstacles()
        with col2:
            if st.button("📂 加载"):
                load_obstacles()
        if st.button("🗑️ 清除全部障碍物"):
            st.session_state.obstacles = []
            st.session_state.drawn_polygon = None
            st.rerun()

        st.subheader("➕ 添加障碍物")
        st.info("1. 点地图工具栏 ⏢ 画多边形\n2. 双击结束\n3. 填名称高度点添加")

        new_name = st.text_input("障碍物名称")
        new_h = st.number_input("高度(m)", 0, 200, 30)
        if st.button("✅ 添加已画多边形"):
            if st.session_state.drawn_polygon and len(st.session_state.drawn_polygon)>=3:
                st.session_state.obstacles.append({
                    "name": new_name or "障碍物",
                    "coords": st.session_state.drawn_polygon,
                    "height": new_h
                })
                st.success("添加成功")
                st.session_state.drawn_polygon = None
                st.rerun()
            else:
                st.error("先在地图画多边形")

    # 坐标转换
    if not is_gcj02:
        lon_a, lat_a = wgs84_to_gcj02(lon_a, lat_a)
        lon_b, lat_b = wgs84_to_gcj02(lon_b, lat_b)

    # 显示地图
    m = create_map(lat_a, lon_a, lat_b, lon_b, st.session_state.obstacles, height)
    output = st_folium(m, width=900, height=600, key="map")

    # 【修复】正确捕获多边形
    if output and output.get("last_active_drawing"):
        draw = output["last_active_drawing"]
        if draw["geometry"]["type"] == "Polygon":
            coords = draw["geometry"]["coordinates"][0]
            st.session_state.drawn_polygon = [[p[0], p[1]] for p in coords]
            st.success(f"✅ 捕获多边形 ({len(coords)}点)")

    st.markdown("🟢起点 🔴终点 🟠障碍物")

# ==================== 飞行监控页面 ====================
else:
    st.title("📡 飞行监控")
    with st.sidebar:
        st.subheader("心跳控制")
        if st.button("▶️ 开始"):
            st.session_state.running = True
            st.session_state.last_time = time.time()
        if st.button("⏹️ 停止"):
            st.session_state.running = False
        if st.button("🗑️ 清空数据"):
            st.session_state.heartbeats = []

    # 心跳生成
    if st.session_state.running:
        if time.time() - st.session_state.last_time >= 1:
            now = datetime.now()
            st.session_state.heartbeats.append({
                "序号": len(st.session_state.heartbeats)+1,
                "时间": now.strftime("%H:%M:%S"),
                "间隔": round(time.time()-st.session_state.last_time,2)
            })
            st.session_state.last_time = time.time()
            st.rerun()

    # 状态
    st.subheader("📊 状态")
    cols = st.columns(4)
    with cols[0]:
        st.metric("心跳总数", len(st.session_state.heartbeats))
    with cols[1]:
        last = st.session_state.heartbeats[-1]["间隔"] if st.session_state.heartbeats else "-"
        st.metric("最后间隔", f"{last}s")
    with cols[2]:
        status = "✅ 在线" if st.session_state.running and st.session_state.heartbeats else "❌ 离线"
        st.metric("连接状态", status)

    # 记录
    st.subheader("心跳记录")
    if st.session_state.heartbeats:
        st.dataframe(st.session_state.heartbeats, use_container_width=True)
    else:
        st.info("点开始生成心跳")
        
