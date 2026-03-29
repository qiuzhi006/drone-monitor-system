import streamlit as st
import pandas as pd
import time
import pydeck as pdk
import math
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

# ==================== 初始化数据 ====================
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

# ==================== 侧边栏导航 ====================
with st.sidebar:
    st.title("🚁 导航")
    page = st.radio("功能页面", ["飞行监控", "航线规划"])
    st.session_state.page = page

# ==================== 障碍物数据 ====================
OBSTACLES = [
    {"name": "教学楼1", "lat": 32.2320, "lon": 118.7488, "height": 30},
    {"name": "教学楼2", "lat": 32.2332, "lon": 118.7490, "height": 35},
    {"name": "图书馆", "lat": 32.2340, "lon": 118.7492, "height": 25},
    {"name": "食堂", "lat": 32.2348, "lon": 118.7495, "height": 20},
    {"name": "宿舍楼", "lat": 32.2355, "lon": 118.7498, "height": 28},
]

# ==================== 航线规划页面 ====================
if st.session_state.page == "航线规划":
    st.title("🗺️ 航线规划 - 3D校园地图")

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

        st.divider()
        st.header("✈️ 飞行参数")
        flight_height = st.slider("飞行高度 (m)", 20, 100, st.session_state.flight_height)
        st.session_state.flight_height = flight_height

        st.divider()
        st.subheader("📌 系统状态")
        col1, col2 = st.columns(2)
        with col1:
            st.success("A点已设")
        with col2:
            st.success("B点已设")
        st.caption(f"当前坐标系: {coord_system}")

    # 坐标转换
    if is_gcj02:
        lat_a_display, lon_a_display = lat_a_input, lon_a_input
        lat_b_display, lon_b_display = lat_b_input, lon_b_input
    else:
        lon_a_display, lat_a_display = wgs84_to_gcj02(lon_a_input, lat_a_input)
        lon_b_display, lat_b_display = wgs84_to_gcj02(lon_b_input, lat_b_input)

    # 保存
    st.session_state.coords_a = {"lat": lat_a_display, "lon": lon_a_display}
    st.session_state.coords_b = {"lat": lat_b_display, "lon": lon_b_display}

    # 3D地图
    st.subheader("🗺️ 校园3D地图")

    path_df = pd.DataFrame({
        "lat": [lat_a_display, lat_b_display],
        "lon": [lon_a_display, lon_b_display]
    })

    obstacle_df = pd.DataFrame(OBSTACLES)

    points_df = pd.DataFrame([
        {"lat": lat_a_display, "lon": lon_a_display, "type": "起点A", "color": [0, 255, 0]},
        {"lat": lat_b_display, "lon": lon_b_display, "type": "终点B", "color": [255, 0, 0]}
    ])

    view_state = pdk.ViewState(
        latitude=(lat_a_display + lat_b_display) / 2,
        longitude=(lon_a_display + lon_b_display) / 2,
        zoom=15,
        pitch=60,
        bearing=0
    )

    path_layer = pdk.Layer(
        "LineLayer",
        data=path_df,
        get_source_position="[lon, lat]",
        get_target_position="[lon, lat]",
        get_color="[255, 0, 0, 200]",
        get_width=5
    )

    obstacle_layer = pdk.Layer(
        "ColumnLayer",
        data=obstacle_df,
        get_position="[lon, lat]",
        get_elevation="height",
        elevation_scale=1,
        radius=30,
        get_fill_color="[255, 100, 0, 150]"
    )

    point_layer = pdk.Layer(
        "ScatterplotLayer",
        data=points_df,
        get_position="[lon, lat]",
        get_color="color",
        get_radius=20
    )

    r = pdk.Deck(
        layers=[path_layer, obstacle_layer, point_layer],
        initial_view_state=view_state,
        tooltip={"text": "{name}"},
        map_style="mapbox://styles/mapbox/satellite-streets-v12"
    )

    st.pydeck_chart(r, use_container_width=True)

    col1, col2, col3, col4 = st.columns(4)
    with col1:
        st.markdown("🟢 **绿色** = 起点A")
    with col2:
        st.markdown("🔴 **红色** = 终点B")
    with col3:
        st.markdown("🟠 **橙色柱** = 障碍物")
    with col4:
        st.markdown("🔴 **红线** = 航线")

    st.divider()
    st.subheader("📐 坐标信息")
    col1, col2 = st.columns(2)
    with col1:
        st.info(f"**起点A** (GCJ-02)\n- 纬度: {lat_a_display:.6f}\n- 经度: {lon_a_display:.6f}")
    with col2:
        st.info(f"**终点B** (GCJ-02)\n- 纬度: {lat_b_display:.6f}\n- 经度: {lon_b_display:.6f}")

    st.caption(f"飞行高度: {flight_height} 米 | 障碍物数量: {len(OBSTACLES)} 个")

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
