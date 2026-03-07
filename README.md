# 12×12 压力传感阵列 - 3D 实时热力图可视化

3D 立体压力分布可视化系统，支持实时数据接收、多种交互方式和平滑动画渲染。

## 运行

- **Processing 4.x IDE**：打开 `Array_Visualization.pde` 并点击 **Run** 按钮（或 Ctrl+R）
- **依赖**：Processing 内置 Serial 库，无需额外安装

---

## 系统架构

### 数据流

```
Teensy 4.1 (Arduino)
  ↓ 串口输出 115200 baud
  "=== 12x12 矩阵 ..." (帧开始标记)
  "R01\t val1\t val2\t...\t val12" (12 行数据)
  "========================" (帧结束标记)
  ↓
Processing
  ↓ parseRowData() 解析行数据
  ↓ commitFrame() 提交完整帧
  ↓ updateDisplayData() 平滑插值 (lerp)
  ↓ drawHeatmap3D() 3D 渲染
```

### 3D 可视化

- **基面网格**：由 13×13 格点构成，每个格点的 Z 高度根据相邻 4 个单元格的平均压力计算
- **表面单元格**：每个 12×12 单元格由两个三角形组成，颜色根据该单元格压力值映射
- **行列标签**：R01~R12（行）和 C01~C12（列）标签在 3D 空间中显示
- **光照效果**：方向光 + 环境光，增强立体感

---

## 颜色映射

使用 **HSB 色彩模式** 实现绿 → 黄 → 红 渐变：

| 参数 | 无压力 (0) | 满载 (30) | 说明 |
|------|-----------|----------|------|
| **Hue** | 120° (绿) | 0° (红) | 色相线性过渡 |
| **Saturation** | 60% | 100% | 低压较浅，高压饱满 |
| **Brightness** | 100% (亮) | 70% (暗) | 增强视觉对比 |

### 映射代码

```processing
float t = map(pressure, PRESSURE_MIN, PRESSURE_MAX, 0.0, 1.0);

float h = map(t, 0.0, 1.0, 120.0, 0.0);      // 色相
float s = map(t, 0.0, 1.0, 60.0, 100.0);     // 饱和度
float b = map(t, 0.0, 1.0, 100.0, 70.0);     // 明度

colorMode(HSB, 360, 100, 100);
color c = color(h, s, b);
colorMode(RGB, 255);
```

---

## 电压 → 压力映射

Arduino 输出的模拟电压线性转换为压力值：

```processing
final float VOLTAGE_MIN = 0.0;      // Arduino 最小电压
final float VOLTAGE_MAX = 3.3;      // Arduino 最大电压
final float PRESSURE_MIN = 0.0;     // 对应最小压力
final float PRESSURE_MAX = 30.0;    // 对应最大压力
```

映射公式：`pressure = map(voltage, VOLTAGE_MIN, VOLTAGE_MAX, PRESSURE_MIN, PRESSURE_MAX)`

**修改范围示例**：若 Arduino 直接输出 0~30 的压力值：

```processing
final float VOLTAGE_MIN = 0.0;
final float VOLTAGE_MAX = 30.0;
final float PRESSURE_MIN = 0.0;
final float PRESSURE_MAX = 30.0;
```

---

## 交互控制

### 鼠标控制

| 操作 | 功能 |
|------|------|
| **左键拖拽** | 旋转 3D 视角（绕 X/Y 轴） |
| **鼠标滚轮** | 放大/缩小（Zoom） |
| **方向键** | 上下左右平移视角 |

### 键盘快捷键

| 快捷键 | 功能 |
|--------|------|
| **D** | 切换演示数据（高斯分布 ↔ 环形分布），无串口时有用 |
| **R** | 重置视角和数据 |
| **S** | 保存当前帧截图（`heatmap_####.png`） |

---

## 配置参数

### 尺寸与布局（[Array_Visualization.pde:34-36](Array_Visualization.pde#L34-L36)）

```processing
final int CELL_SIZE = 50;          // 每个网格单元的像素宽度
final int PADDING = 60;            // 画布边距
final int LEGEND_WIDTH = 30;       // 色条宽度
final int LEGEND_GAP = 30;         // 色条与网格间距
```

### 压力范围（[Array_Visualization.pde:25-27](Array_Visualization.pde#L25-L27)）

```processing
final float PRESSURE_MIN = 0.0;     // 最小压力显示范围
final float PRESSURE_MAX = 30.0;    // 最大压力显示范围
final float PRESSURE_HEIGHT_MAX = 150.0;  // 3D 高度缩放因子
```

### 串口设置（[Array_Visualization.pde:40-41](Array_Visualization.pde#L40-L41)）

```processing
final int BAUD_RATE = 115200;
String SERIAL_PORT_NAME = "";  // 空字符串表示自动选择最后一个串口
```

若要指定特定端口（如 COM3），改为：

```processing
String SERIAL_PORT_NAME = "COM3";
```

---

## 界面元素

### 状态栏（右下角）
- 串口连接状态（Connected / Disconnected）
- 数据帧率 (Data FPS)
- 压力范围
- 窗口分辨率

### 色条图例（右侧）
- 压力值渐变色
- 6 个等间距刻度标签（0~30）
- "Pressure" 标题

### 操作提示（左上角）
- 快捷键列表和控制说明

---

## 演示模式

按 **D** 键切换演示数据（无 Teensy 连接时用于调试）：

1. **高斯分布**：以中心为峰值，向四周衰减的钟形分布
2. **环形分布**：压力集中在特定半径的环带周围

每次按 D 切换一种分布模式。

---

## 特性

- ✅ **3D 立体渲染**：曲面网格根据压力动态变形
- ✅ **平滑动画**：使用 lerp 插值平滑过渡数据变化
- ✅ **自适应缩放**：响应式布局，支持窗口拖拽调整
- ✅ **双缓冲**：`bufferData[]` 暂存中间数据，`pressureData[]` 存储完整帧
- ✅ **鲁棒串口解析**：支持中断接收、行数据校验、范围约束
- ✅ **光照效果**：方向光 + 环境光增强 3D 视觉效果
