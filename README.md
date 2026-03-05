# 12×12 压力传感阵列 - 实时热力图可视化

## 依赖

- **Processing Serial Library** — Processing 内置，无需额外安装。打开 Processing IDE 直接运行即可。

---

## 颜色映射核心逻辑

### HSB 色彩模式详解

使用 `colorMode(HSB, 360, 100, 100)` 实现绿色 → 红色的平滑过渡。

| 参数 | 低压 (0) | 高压 (30) | 说明 |
|------|----------|----------|------|
| **Hue** | 120° (绿) | 0° (红) | 色相从绿过渡到红 |
| **Saturation** | 25% (淡) | 100% (浓) | 低压浅淡，高压饱满 |
| **Brightness** | 100% (亮) | 70% (暗) | 低压明亮，高压深沉 |

**综合效果**: 压力=0 呈极浅淡绿色，中间经过黄色过渡，压力=30 呈深沉浓郁红色。无效/缺失数据显示为纯白 `#FFFFFF`。

### 映射公式

```processing
float t = map(pressure, 0, 30, 0.0, 1.0);           // 归一化

float h = map(t, 0.0, 1.0, 120.0, 0.0);             // 色相
float s = map(t, 0.0, 1.0, 25.0, 100.0);            // 饱和度
float b = map(t, 0.0, 1.0, 100.0, 70.0);            // 明度

colorMode(HSB, 360, 100, 100);
color c = color(h, s, b);
colorMode(RGB, 255);
```

---

## 电压 → 压力映射

Arduino 输出电压 0~3.3V，代码通过以下参数线性映射到压力 0~30：

```processing
final float VOLTAGE_MIN = 0.0;     // Arduino 最小输出电压
final float VOLTAGE_MAX = 3.3;     // Arduino 最大输出电压
final float PRESSURE_MIN = 0.0;    // 对应的最小压力值
final float PRESSURE_MAX = 30.0;   // 对应的最大压力值
```

**示例**: 如果 Arduino 直接输出压力值（0~30），改为：

```processing
final float VOLTAGE_MIN = 0.0;
final float VOLTAGE_MAX = 30.0;
final float PRESSURE_MIN = 0.0;
final float PRESSURE_MAX = 30.0;
```

---

## 快捷键

| 快捷键 | 功能 |
|--------|------|
| **D** | 填充高斯分布演示数据（无串口时测试） |
| **R** | 重置所有数据 |
| **S** | 保存当前帧为截图 |

---

## 串口选择

默认自动选择最后一个串口。如需指定特定端口，修改代码顶部：

```processing
String SERIAL_PORT_NAME = "COM3";  // 替换为你的端口号
```
