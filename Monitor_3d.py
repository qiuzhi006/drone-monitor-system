import streamlit as st
import pandas as pd
import time
import json
import random
from datetime import datetime

# --- 页面配置 ---
st.set_page_config(page_title="无人机智能规划系统", layout="wide")

# --- 初始化 Session State ---
if 'heartbeats' not in st.session_state:
    st.session_state.heartbeats = []
if 'obstacles' not in st.session_state:
    st.session_state.obstacles = []  # 存储障碍物数据
if 'last_time' not in st.session_state:
    st.session_state.last_time = time.time()
if 'drone_active' not in st.session_state:
    st.session_state.drone_active = False

# --- 辅助函数 ---
def generate_heartbeat():
    """模拟生成心跳包"""
    new_beat = {
        "序号": len(st.session_state.heartbeats) + 1,
        "时间": datetime.now(),
        "延迟(秒)": round(random.uniform(0.1, 0.5), 2),
        "电量": max(0, 100 - len(st.session_state.heartbeats) * 0.1)
    }
    st.session_state.heartbeats.append(new_beat)
    # 限制列表长度防止内存溢出
    if len(st.session_state.heartbeats) > 50:
        st.session_state.heartbeats.pop(0)

# --- 侧边栏：参数设置 ---
st.sidebar.header("🚁 任务参数配置")
flight_altitude = st.sidebar.number_input("设置飞行高度 (米)", min_value=0, max_value=500, value=50, step=5)
safety_radius = st.sidebar.number_input("安全半径 (米)", min_value=1, max_value=50, value=5, step=1)

st.sidebar.divider()
st.sidebar.subheader("🎮 控制台")
if st.sidebar.button("🟢 启动无人机模拟"):
    st.session_state.drone_active = True
    st.session_state.heartbeats = []  # 重置心跳
if st.sidebar.button("🔴 停止模拟"):
    st.session_state.drone_active = False

# --- 主界面 ---
st.title("🛰️ 无人机智能化应用 Demo")
st.markdown("### 1. 地图显示与障碍物圈选")
st.info("💡 **操作指南：** 使用左侧工具栏的 **多边形工具** 在地图上绘制障碍物。绘制完成后，在下方输入高度并点击“添加障碍物”。")

# 地图组件
try:
    from streamlit_folium import st_folium
    import folium
    from folium.plugins import Draw

    # 初始化地图
    m = folium.Map(location=[31.2304, 121.4737], zoom_start=15)  # 默认上海

    # 添加绘制控件
    draw = Draw(
        export=False,
        draw_options={
            "polyline": False,
            "rectangle": False,
            "circle": False,
            "marker": False,
            "circlemarker": False,
        }
    )
    draw.add_to(m)

    # 渲染地图并获取绘制数据
    map_data = st_folium(m, width=700, height=450, key="map_selector")

    # 处理新绘制的障碍物
    if map_data["last_active_drawing"]:
        # 检查是否已经存在这个障碍物（防止重复添加）
        geojson = map_data["last_active_drawing"]
        is_new = True
        for obs in st.session_state.obstacles:
            if obs['geojson'] == geojson:
                is_new = False
                break

        if is_new:
            st.session_state.new_obstacle_geo = geojson
            st.success("✅ 检测到新图形！请在下方设置高度并添加。")
    else:
        st.session_state.new_obstacle_geo = None

except ImportError:
    st.error("缺少库：请运行 `pip install streamlit-folium folium`")

# --- 障碍物管理 ---
st.markdown("### 2. 障碍物管理")

# 如果有新绘制的图形，显示输入框
if st.session_state.get('new_obstacle_geo'):
    with st.form("add_obs_form"):
        obs_height = st.number_input("设置障碍物高度 (米)", min_value=0, value=30)
        submitted = st.form_submit_button("添加到障碍物列表")
        if submitted:
            new_obs = {
                "id": len(st.session_state.obstacles) + 1,
                "height": obs_height,
                "geojson": st.session_state.new_obstacle_geo
            }
            st.session_state.obstacles.append(new_obs)
            st.session_state.new_obstacle_geo = None  # 清空暂存
            st.rerun()

# 显示现有障碍物列表
if st.session_state.obstacles:
    obs_df = pd.DataFrame([
        {"ID": o["id"], "高度(米)": o["height"], "类型": o["geojson"]["geometry"]["type"]}
        for o in st.session_state.obstacles
    ])
    st.dataframe(obs_df, use_container_width=True)

    # JSON 导出功能
    def convert_df_to_json(df):
        return json.dumps([o for o in st.session_state.obstacles], indent=4, default=str)

    json_data = convert_df_to_json(obs_df)
    st.download_button(
        label="💾 一键保存障碍物为 JSON",
        data=json_data,
        file_name='obstacles.json',
        mime='application/json'
    )
else:
    st.info("暂无障碍物，请在地图上绘制。")

st.divider()

# --- 3. 航线规划与状态监控 ---
st.markdown("### 3. 航线规划与实时状态")

col_map, col_status = st.columns([2, 1])

with col_status:
    st.subheader("📡 实时遥测")

    # --- 心跳逻辑 ---
    current_time = time.time()
    # 如果无人机激活，自动生成心跳
    if st.session_state.drone_active:
        if current_time - st.session_state.last_time >= 1:
            generate_heartbeat()
            st.session_state.last_time = current_time
            st.rerun()

    # --- 状态显示 ---
    if len(st.session_state.heartbeats) > 0:
        latest = st.session_state.heartbeats[-1]
        last_beat_time = latest["时间"].timestamp()
        seconds_since = time.time() - last_beat_time

        st.metric("最新心跳序号", latest["序号"])
        st.metric("信号延迟", f"{latest['延迟(秒)']} 秒")

        if seconds_since > 3:
            st.error(f"⚠️ 掉线 ({seconds_since:.1f}s)")
        else:
            st.success("✅ 连接正常")

        st.progress(latest['电量'] / 100, text=f"电量 {latest['电量']:.1f}%")

    else:
        st.info("等待启动...")

with col_map:
    st.subheader("🗺️ 航线规划决策")

    if not st.session_state.obstacles:
        st.warning("请先添加障碍物以进行规划。")
    else:
        # 模拟规划逻辑
        st.write(f"**当前设定飞行高度：** `{flight_altitude}米`")
        st.write(f"**安全半径：** `{safety_radius}米`")
        st.write("---")

        plan_data = []
        for obs in st.session_state.obstacles:
            obs_h = obs['height']
            # 核心逻辑：高度比较
            if flight_altitude > obs_h:
                action = "🚀 直接飞跃"
                color = "green"
                detail = f"高度充裕 (差值: {flight_altitude - obs_h}m)"
            else:
                action = "🔄 自动绕行"
                color = "orange"
                detail = f"高度不足，计算避障路径..."

            plan_data.append({
                "障碍物ID": obs['id'],
                "障碍物高度": obs_h,
                "决策动作": action,
                "详情": detail
            })

        plan_df = pd.DataFrame(plan_data)
        st.dataframe(plan_df, use_container_width=True)

        # 简单的图表展示
        st.bar_chart(plan_df.set_index('障碍物ID')['障碍物高度'])

# --- 底部：心跳趋势 ---
st.divider()
st.subheader("📈 历史心跳数据")
if len(st.session_state.heartbeats) > 0:
    df_hist = pd.DataFrame(st.session_state.heartbeats)
    st.line_chart(df_hist.set_index('时间')['序号'])
else:
    st.write("无历史数据")
