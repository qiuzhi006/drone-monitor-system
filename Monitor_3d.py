import streamlit as st
import folium
from folium import plugins
from streamlit_folium import st_folium
import json
import math
import os
import random
from datetime import datetime
# --- 新增：用于几何计算 ---
from shapely.geometry import Point, Polygon, LineString

# 页面配置
st.set_page_config(page_title="无人机航线规划系统", layout="wide")

# --- 初始化 Session State ---
if 'obstacles' not in st.session_state:
    st.session_state.obstacles = []
if 'flight_data' not in st.session_state:
    st.session_state.flight_data = {
        "start": [31.2304, 121.4737],
        "end": [31.2400, 121.4800],
        "status": "未规划",
        "uav_pos": [31.2304, 121.4737], # 无人机实时位置
        "battery": 100,
        "signal": 95
    }

# --- 工具函数 ---
def gcj02_to_wgs84(lng, lat):
    """
    GCJ-02 to WGS-84 coordinate conversion
    """
    pi = 3.1415926535897932384626
    a = 6378245.0
    ee = 0.00669342162296594323

    def transform_lat(x, y):
        ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * pi) + 20.0 * math.sin(2.0 * x * pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(y * pi) + 40.0 * math.sin(y / 3.0 * pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(y / 12.0 * pi) + 320 * math.sin(y * pi / 30.0)) * 2.0 / 3.0
        return ret

    def transform_lng(lng, lat):
        ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * pi) + 20.0 * math.sin(2.0 * x * pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(x * pi) + 40.0 * math.sin(x / 3.0 * pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(x / 12.0 * pi) + 300.0 * math.sin(x / 30.0 * pi)) * 2.0 / 3.0
        return ret

    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * pi)
    mglat = lat + dlat
    mglng = lng + dlng
    return lng * 2 - mglng, lat * 2 - mglat

# --- 新增：避障计算核心逻辑 ---
def calculate_avoidance_paths(start, end, obstacles, flight_altitude, safety_radius_m):
    """
    计算三种航线方案
    start/end: (lat, lon)
    obstacles: st.session_state.obstacles 格式
    """
    # 默认方案就是直线
    paths = {
        "optimal": [start, end],
        "left": [],
        "right": []
    }

    # 1. 高度判断逻辑 (作业核心要求)
    # 检查是否有障碍物高于飞行高度
    high_obstacles = [obs for obs in obstacles if obs['height'] >= flight_altitude]

    if not high_obstacles:
        # 如果没有障碍物高于飞行高度，直接飞直线
        return {
            "status": "safe",
            "paths": paths,
            "msg": "高度安全，无障碍物阻挡"
        }

    # 2. 平面避障逻辑 (如果有障碍物高于飞行高度)
    # 简单模拟：如果直线穿过障碍物，生成绕行点
    line = LineString([start, end])

    blocked = False
    for obs in high_obstacles:
        # 创建障碍物多边形 (简化为以坐标为中心的圆或方)
        # 这里简化处理：假设障碍物是一个以 coords[0] 为中心，半径为 50米的圆
        center = Point(obs['coords'][0][1], obs['coords'][0][0]) # lon, lat
        buffer = center.buffer(0.0005) # 约50米

        if line.intersects(buffer):
            blocked = True
            # 简单的绕行点生成 (实际应使用更复杂的算法如A*或可视图法)
            # 这里为了演示，我们在障碍物侧边生成一个偏移点
            mid_point = [(start[0] + end[0])/2, (start[1] + end[1])/2]

            # 左绕 (向北/东偏移)
            paths["left"] = [start, [mid_point[0]+0.001, mid_point[1]+0.001], end]
            # 右绕 (向南/西偏移)
            paths["right"] = [start, [mid_point[0]-0.001, mid_point[1]-0.001], end]

    if not blocked:
        return {"status": "safe", "paths": paths, "msg": "路径畅通"}
    else:
        return {"status": "unsafe", "paths": paths, "msg": "检测到障碍物，建议绕行"}

# --- 侧边栏：参数设置 ---
with st.sidebar:
    st.header("🚁 飞行参数设置")

    # 飞行高度输入
    flight_altitude = st.number_input("设定飞行高度 (米)", min_value=10, max_value=500, value=50, step=5)
    safety_radius = st.slider("安全半径 (米)", 10, 100, 30)

    st.divider()

    st.header("🗺️ 地图控制")
    if st.button("重置地图视图"):
        st.rerun()

# --- 主界面 ---
st.title("🛰️ 智能无人机三维航线规划系统")

# 选项卡
tab1, tab2, tab3 = st.tabs(["🗺️ 航线规划", "🚧 障碍物管理", "📊 状态监控"])

with tab1:
    st.subheader("1. 设置起终点")
    col1, col2 = st.columns(2)

    with col1:
        st.markdown("**起点 (Start)**")
        lat_a_input = st.number_input("起点纬度", value=31.2304, key="lat_a")
        lon_a_input = st.number_input("起点经度", value=121.4737, key="lon_a")

    with col2:
        st.markdown("**终点 (End)**")
        lat_b_input = st.number_input("终点纬度", value=31.2400, key="lat_b")
        lon_b_input = st.number_input("终点经度", value=121.4800, key="lon_b")

    start_point = (lat_a_input, lon_a_input)
    end_point = (lat_b_input, lon_b_input)

    # 更新飞行数据
    st.session_state.flight_data['start'] = start_point
    st.session_state.flight_data['end'] = end_point

    st.divider()

    st.subheader("2. 航线生成与分析")

    # 模拟心跳监测逻辑 (为了演示，我们在每次重绘时稍微更新一下状态)
    # 实际应用中这应该是一个后台线程或API调用
    if st.session_state.flight_data['status'] == "飞行中":
        # 简单模拟无人机移动
        current_lat = st.session_state.flight_data['uav_pos'][0]
        current_lon = st.session_state.flight_data['uav_pos'][1]
        target_lat = end_point[0]
        target_lon = end_point[1]

        # 简单的线性插值移动
        if abs(current_lat - target_lat) > 0.0001:
            step = 0.0001
            new_lat = current_lat + step if current_lat < target_lat else current_lat - step
            new_lon = current_lon + step if current_lon < target_lon else current_lon - step
            st.session_state.flight_data['uav_pos'] = [new_lat, new_lon]
            st.session_state.flight_data['battery'] -= 0.1

    # 地图初始化
    m = folium.Map(location=[(lat_a_input + lat_b_input) / 2, (lon_a_input + lon_b_input) / 2], zoom_start=15, tiles="OpenStreetMap")

    # 绘制起点和终点
    folium.Marker(start_point, popup="起点", icon=folium.Icon(color="green", icon="play")).add_to(m)
    folium.Marker(end_point, popup="终点", icon=folium.Icon(color="red", icon="stop")).add_to(m)

    # 绘制障碍物
    for obs in st.session_state.obstacles:
        # 提取坐标
        coords = [(c[0], c[1]) for c in obs['coords']]
        # 绘制多边形
        folium.Polygon(
            locations=coords,
            color="red",
            weight=2,
            fill=True,
            fill_color="red",
            fill_opacity=0.4,
            popup=f"{obs['name']} (高: {obs['height']}m)"
        ).add_to(m)

        # 绘制高度标注
        center_lat = sum(c[0] for c in coords) / len(coords)
        center_lon = sum(c[1] for c in coords) / len(coords)
        folium.Marker(
            [center_lat, center_lon],
            icon=folium.DivIcon(html=f"""<div style="font-size: 12px; color: red; background: white; border-radius: 5px; padding: 2px;">{obs['height']}m</div>""")
        ).add_to(m)

    # 航线生成按钮逻辑
    if st.button("生成航线", key="generate_route_btn"):
        # 调用避障算法
        result = calculate_avoidance_paths(
            start_point,
            end_point,
            st.session_state.obstacles,
            flight_altitude,
            safety_radius
        )

        st.session_state.flight_data['status'] = "已规划"
        st.session_state.flight_data['paths'] = result['paths']

        # 绘制最佳航线 (绿色)
        if result['paths']['optimal']:
            folium.PolyLine(
                result['paths']['optimal'],
                color="green",
                weight=5,
                opacity=0.8,
                popup="最佳航线"
            ).add_to(m)

        # 绘制备选航线 (虚线)
        if result['paths']['left']:
            folium.PolyLine(
                result['paths']['left'],
                color="orange",
                weight=3,
                dash_array="10, 10",
                opacity=0.6,
                popup="左侧绕行方案"
            ).add_to(m)

        if result['paths']['right']:
            folium.PolyLine(
                result['paths']['right'],
                color="blue",
                weight=3,
                dash_array="10, 10",
                opacity=0.6,
                popup="右侧绕行方案"
            ).add_to(m)

        # 显示分析结果
        if result['status'] == 'safe':
            st.success(f"✅ **航线安全**：{result['msg']}")
        else:
            st.warning(f"⚠️ **注意**：{result['msg']}，请选择绕行方案。")

    # 如果已经规划过，重新绘制存储的航线
    elif 'paths' in st.session_state.flight_data:
        paths = st.session_state.flight_data['paths']
        # 绘制最佳
        if paths['optimal']:
            folium.PolyLine(paths['optimal'], color="green", weight=5, opacity=0.8).add_to(m)
        # 绘制备选
        if paths['left']:
            folium.PolyLine(paths['left'], color="orange", weight=3, dash_array="10, 10", opacity=0.6).add_to(m)
        if paths['right']:
            folium.PolyLine(paths['right'], color="blue", weight=3, dash_array="10, 10", opacity=0.6).add_to(m)

    # 绘制无人机实时位置 (心跳监测可视化)
    if st.session_state.flight_data['status'] == "飞行中":
        uav_pos = st.session_state.flight_data['uav_pos']
        folium.Marker(
            uav_pos,
            popup="无人机实时位置",
            icon=folium.Icon(color="purple", icon="plane")
        ).add_to(m)
        # 添加动态效果 (简单的脉冲圆)
        plugins.CircleMarker(
            uav_pos,
            radius=10,
            color="purple",
            fill=True,
            fill_opacity=0.2
        ).add_to(m)

    # 显示地图
    st_data = st_folium(m, width=700, height=500)

with tab2:
    st.subheader("障碍物管理")

    with st.form("add_obstacle_form"):
        st.markdown("### 添加新障碍物")
        obs_name = st.text_input("障碍物名称")
        obs_height = st.number_input("障碍物高度 (米)", min_value=1, value=30)

        # 简单的多边形输入 (实际应用中可以使用地图点击绘制)
        st.markdown("顶点坐标 (纬度,经度)，每行一个点:")
        default_coords = "31.235, 121.475\n31.236, 121.476\n31.234, 121.477"
        coords_input = st.text_area("坐标输入", value=default_coords, height=100)

        submitted = st.form_submit_button("添加障碍物")
        if submitted:
            try:
                lines = coords_input.strip().split('\n')
                coords = []
                for line in lines:
                    lat_str, lon_str = line.split(',')
                    coords.append([float(lat_str.strip()), float(lon_str.strip())])

                if len(coords) >= 3:
                    new_obs = {
                        "name": obs_name,
                        "height": obs_height,
                        "coords": coords
                    }
                    st.session_state.obstacles.append(new_obs)
                    st.success(f"已添加障碍物：{obs_name}")
                    st.rerun()
                else:
                    st.error("至少需要3个顶点")
            except Exception as e:
                st.error(f"坐标格式错误: {e}")

    st.divider()
    st.subheader("当前障碍物列表")
    if not st.session_state.obstacles:
        st.info("暂无障碍物")
    else:
        for i, obs in enumerate(st.session_state.obstacles):
            with st.expander(f"{obs['name']} (高: {obs['height']}m)", expanded=False):
                st.write(f"顶点数: {len(obs['coords'])}")
                if st.button(f"删除 {obs['name']}", key=f"del_{i}"):
                    st.session_state.obstacles.pop(i)
                    st.rerun()

with tab3:
    st.subheader("无人机实时状态 (心跳监测)")

    # 模拟数据更新
    if st.button("启动模拟飞行"):
        st.session_state.flight_data['status'] = "飞行中"
        st.session_state.flight_data['battery'] = 100
        st.rerun()

    if st.button("停止飞行"):
        st.session_state.flight_data['status'] = "已停止"
        st.rerun()

    col1, col2, col3 = st.columns(3)
    data = st.session_state.flight_data

    col1.metric("飞行状态", data['status'], delta=None)
    col2.metric("剩余电量", f"{data['battery']:.1f}%", delta="-0.1%" if data['status']=="飞行中" else "0%")
    col3.metric("信号强度", f"{data['signal']}%", delta=None)

    st.markdown("### 实时位置信息")
    st.json({
        "纬度": data['uav_pos'][0],
        "经度": data['uav_pos'][1],
        "时间戳": datetime.now().strftime("%H:%M:%S")
    })

    # 简单的图表模拟
    st.markdown("### 历史轨迹 (模拟)")
    # 这里可以用 st.line_chart 如果有真实数据流
    chart_data = {
        "纬度": [data['start'][0], data['uav_pos'][0]],
        "经度": [data['start'][1], data['uav_pos'][1]]
    }
    st.line_chart(chart_data)
