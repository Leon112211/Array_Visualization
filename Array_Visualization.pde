/***************************************************************
 * 项目：12x12 压力传感阵列 - 实时热力图可视化
 * 平台：Processing 4.x
 * 依赖：Processing Serial Library (内置，无需额外安装)
 *
 * 功能：
 *   - 通过串口接收 Arduino/Teensy 发送的 12x12 矩阵数据
 *   - 将压力数值实时转化为热力图 (绿色 -> 红色)
 *   - 缺失/无效数据显示为白色
 *
 * 数据协议：
 *   Arduino 端以 "R01\t值1\t值2\t...\t值12" 格式逐行输出
 *   每帧由 "===" 开头的行标记起始/结束
 ***************************************************************/

import processing.serial.*;

// ==================== 系统配置 ====================
final int ROWS = 12;
final int COLS = 12;

// 压力数值范围：0 = 无压力，30 = 满载压力
final float PRESSURE_MIN = 0.0;
final float PRESSURE_MAX = 30.0;

// 电压 -> 压力 的线性映射参数
// Arduino 输出电压范围 0~3.3V，对应压力 0~30
// 如果 Arduino 直接输出压力值，将这两个值设为与 PRESSURE 相同即可
final float VOLTAGE_MIN = 0.0;
final float VOLTAGE_MAX = 3.3;

// 网格绘制参数
final int CELL_SIZE = 50;          // 每个方格的像素大小
final int PADDING = 60;            // 画布四周留白
final int LABEL_OFFSET = 15;       // 行/列标签偏移量
final int LEGEND_WIDTH = 30;       // 色条图例宽度
final int LEGEND_GAP = 30;         // 色条与网格的间距

// 串口配置
final int BAUD_RATE = 115200;
// 如果需要指定串口名称，在此修改；留空则自动选择第一个可用串口
String SERIAL_PORT_NAME = "";

// ==================== 全局变量 ====================
Serial serialPort;

// 数据矩阵：存储当前帧的压力值
// NaN 表示该点数据无效/缺失 -> 绘制为白色
float[][] pressureData = new float[ROWS][COLS];

// 临时缓冲区：在接收完整帧期间暂存数据
float[][] bufferData = new float[ROWS][COLS];
boolean[] rowReceived = new boolean[ROWS];  // 标记哪些行已收到
boolean frameInProgress = false;            // 是否正在接收一帧数据

// 帧计数器（用于显示统计）
int frameCount = 0;
long lastFrameTime = 0;
float fps = 0;

// ==================== Processing 入口 ====================

void setup() {
  // 窗口大小 = 网格 + 留白 + 色条图例
  // 宽度：左留白 + 12格 + 右留白 + 色条区域
  // 高度：上留白 + 12格 + 下留白
  int windowWidth  = PADDING + COLS * CELL_SIZE + LEGEND_GAP + LEGEND_WIDTH + PADDING;
  int windowHeight = PADDING + ROWS * CELL_SIZE + PADDING;

  // Processing 4.x 中 size() 必须使用字面常量，所以改用 surface
  surface.setSize(windowWidth, windowHeight);
  surface.setTitle("12x12 Pressure Array Heatmap");

  // 初始化数据为 NaN（无效）
  initDataArrays();

  // 初始化串口
  initSerial();

  // 设置文本属性
  textAlign(CENTER, CENTER);
  textSize(12);

  println("=== 可视化系统就绪 ===");
  println("窗口大小: " + windowWidth + " x " + windowHeight);
  println("压力范围: " + PRESSURE_MIN + " ~ " + PRESSURE_MAX);
}

void draw() {
  background(40);  // 深灰色背景

  // 处理所有待读取的串口数据
  processSerialData();

  // 绘制网格热力图
  drawHeatmap();

  // 绘制色条图例
  drawColorLegend();

  // 绘制状态信息
  drawStatusBar();
}

// ==================== 数据初始化 ====================

void initDataArrays() {
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      pressureData[r][c] = Float.NaN;
      bufferData[r][c]   = Float.NaN;
    }
    rowReceived[r] = false;
  }
}

// ==================== 串口初始化 ====================

void initSerial() {
  String[] portList = Serial.list();
  println("可用串口列表:");
  for (int i = 0; i < portList.length; i++) {
    println("  [" + i + "] " + portList[i]);
  }

  if (portList.length == 0) {
    println("[警告] 未检测到串口设备，将以演示模式运行");
    return;
  }

  String portName;
  if (SERIAL_PORT_NAME.length() > 0) {
    portName = SERIAL_PORT_NAME;
  } else {
    // 自动选择最后一个串口（通常是最新连接的设备）
    portName = portList[portList.length - 1];
  }

  try {
    serialPort = new Serial(this, portName, BAUD_RATE);
    serialPort.bufferUntil('\n');  // 按行缓冲
    println("已连接串口: " + portName + " @ " + BAUD_RATE + " baud");
  } catch (Exception e) {
    println("[错误] 无法打开串口 " + portName + ": " + e.getMessage());
    serialPort = null;
  }
}

// ==================== 串口数据处理 ====================

/**
 * 持续读取并解析串口缓冲区中的所有数据行。
 * 数据协议:
 *   - "=== 12x12" 开头 -> 新帧开始
 *   - "====" 开头 (不含 "12x12") -> 当前帧结束，提交数据
 *   - "R01\t..." ~ "R12\t..." -> 行数据
 */
void processSerialData() {
  if (serialPort == null) return;

  while (serialPort.available() > 0) {
    String line = serialPort.readStringUntil('\n');
    if (line == null) break;

    line = line.trim();
    if (line.length() == 0) continue;

    // 检测帧起始标记: "=== 12x12 矩阵 ..."
    if (line.startsWith("=== 12x12")) {
      // 新帧开始，重置缓冲区
      frameInProgress = true;
      for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
          bufferData[r][c] = Float.NaN;
        }
        rowReceived[r] = false;
      }
      continue;
    }

    // 检测帧结束标记: "========================"
    if (line.startsWith("====") && !line.contains("12x12")) {
      if (frameInProgress) {
        // 将缓冲区数据提交到显示数据
        commitFrame();
        frameInProgress = false;
      }
      continue;
    }

    // 解析行数据: "R01\t 0.12\t 0.34\t..."
    if (frameInProgress && line.startsWith("R")) {
      parseRowData(line);
    }
  }
}

/**
 * 解析一行数据，格式: "R01\t值1\t值2\t...\t值12"
 * 行号从 R01 到 R12
 */
void parseRowData(String line) {
  String[] parts = line.split("\t");
  if (parts.length < 2) return;

  // 提取行号: "R01" -> 0, "R12" -> 11
  String rowStr = parts[0].trim();
  if (rowStr.length() < 2) return;

  int rowIndex;
  try {
    rowIndex = Integer.parseInt(rowStr.substring(1)) - 1;  // "R01" -> 0
  } catch (NumberFormatException e) {
    return;  // 无效行号
  }

  if (rowIndex < 0 || rowIndex >= ROWS) return;

  // 解析该行的每个数值
  for (int c = 0; c < COLS && c + 1 < parts.length; c++) {
    String valStr = parts[c + 1].trim();
    try {
      float voltage = Float.parseFloat(valStr);

      // ---- 电压 -> 压力 线性映射 ----
      // 将 Arduino 输出的电压值映射到压力范围
      float pressure = map(voltage, VOLTAGE_MIN, VOLTAGE_MAX, PRESSURE_MIN, PRESSURE_MAX);

      // 钳位到有效范围
      pressure = constrain(pressure, PRESSURE_MIN, PRESSURE_MAX);

      bufferData[rowIndex][c] = pressure;
    } catch (NumberFormatException e) {
      // 解析失败，保持 NaN（将显示为白色）
      bufferData[rowIndex][c] = Float.NaN;
    }
  }

  rowReceived[rowIndex] = true;
}

/**
 * 将缓冲区中的完整帧数据提交到显示矩阵。
 * 未收到的行保留 NaN，绘制时显示为白色。
 */
void commitFrame() {
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      pressureData[r][c] = bufferData[r][c];
    }
  }

  // 更新帧率统计
  frameCount++;
  long now = millis();
  if (now - lastFrameTime > 1000) {
    fps = frameCount * 1000.0 / (now - lastFrameTime);
    frameCount = 0;
    lastFrameTime = now;
  }
}

// ==================== 颜色映射 ====================

/**
 * 核心颜色映射函数：将压力值映射为热力图颜色。
 *
 * 【颜色映射逻辑 - colorMode(HSB)】
 *
 * 使用 HSB (色相-饱和度-明度) 颜色模式：
 *   - H (Hue 色相): 0~360 度
 *       绿色 = 120 度, 红色 = 0 度
 *       压力 0 -> H=120 (绿), 压力 30 -> H=0 (红)
 *       使用 map() 将 [0, 30] 线性映射到 [120, 0]（注意是反向映射）
 *
 *   - S (Saturation 饱和度): 0~100%
 *       低压时饱和度低(颜色淡/浅), 高压时饱和度高(颜色浓/深)
 *       使用 map() 将 [0, 30] 映射到 [25, 100]
 *       最低 25% 保留一定底色，避免完全变白
 *
 *   - B (Brightness 明度): 0~100%
 *       低压时高亮度(颜色亮/浅), 高压时适中亮度(颜色深沉)
 *       使用 map() 将 [0, 30] 映射到 [100, 70]（注意是反向映射）
 *       高压端保持 70% 明度，保证红色深沉但不至于全黑
 *
 * 综合效果：
 *   压力=0  -> 极浅淡绿色 (H=120, S=25, B=100)
 *   压力=15 -> 中等饱和的黄色 (H=60, S=62, B=85)
 *   压力=30 -> 深沉浓郁的红色 (H=0, S=100, B=70)
 *
 * @param pressure 压力值 (0~30)
 * @return Processing color 对象
 */
color pressureToColor(float pressure) {
  // 无效数据 -> 纯白色
  if (Float.isNaN(pressure)) {
    return color(255, 255, 255);
  }

  // 确保在有效范围内
  pressure = constrain(pressure, PRESSURE_MIN, PRESSURE_MAX);

  // 归一化到 0.0 ~ 1.0
  float t = map(pressure, PRESSURE_MIN, PRESSURE_MAX, 0.0, 1.0);

  // --- 色相 (Hue): 绿(120) -> 红(0)，线性反向映射 ---
  // map(t, 0, 1, 120, 0) 等价于 120 * (1 - t)
  float h = map(t, 0.0, 1.0, 120.0, 0.0);

  // --- 饱和度 (Saturation): 低压淡(25%) -> 高压浓(100%) ---
  // 低压时颜色接近白色/淡色，高压时颜色饱满浓郁
  float s = map(t, 0.0, 1.0, 25.0, 100.0);

  // --- 明度 (Brightness): 低压亮(100%) -> 高压暗(70%) ---
  // 使低压区域呈现浅亮的淡绿色，高压区域呈现深沉的红色
  float b = map(t, 0.0, 1.0, 100.0, 70.0);

  // 切换到 HSB 模式计算颜色，然后切回 RGB
  colorMode(HSB, 360, 100, 100);
  color c = color(h, s, b);
  colorMode(RGB, 255);

  return c;
}

// ==================== 绘图函数 ====================

/**
 * 绘制 12x12 热力图网格。
 * 每个方格根据对应压力值着色，并在方格内显示数值。
 */
void drawHeatmap() {
  float gridStartX = PADDING;
  float gridStartY = PADDING;

  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      float x = gridStartX + c * CELL_SIZE;
      float y = gridStartY + r * CELL_SIZE;

      // 获取颜色
      color cellColor = pressureToColor(pressureData[r][c]);

      // 绘制方格
      fill(cellColor);
      stroke(60);           // 深灰色边框
      strokeWeight(1.5);
      rect(x, y, CELL_SIZE, CELL_SIZE, 3);  // 圆角矩形

      // 在方格内显示数值
      if (!Float.isNaN(pressureData[r][c])) {
        // 根据背景明暗选择文字颜色
        float brightness_val = brightness(cellColor);
        fill(brightness_val > 50 ? 0 : 255);  // 亮背景用黑字，暗背景用白字
        noStroke();
        textSize(11);
        text(nf(pressureData[r][c], 0, 1), x + CELL_SIZE / 2, y + CELL_SIZE / 2);
      } else {
        // 无效数据显示 "---"
        fill(180);
        noStroke();
        textSize(10);
        text("---", x + CELL_SIZE / 2, y + CELL_SIZE / 2);
      }
    }
  }

  // 绘制行标签 (R01 ~ R12)
  fill(200);
  textSize(11);
  textAlign(RIGHT, CENTER);
  for (int r = 0; r < ROWS; r++) {
    float y = gridStartY + r * CELL_SIZE + CELL_SIZE / 2;
    text("R" + nf(r + 1, 2), gridStartX - 8, y);
  }

  // 绘制列标签 (C01 ~ C12)
  textAlign(CENTER, BOTTOM);
  for (int c = 0; c < COLS; c++) {
    float x = gridStartX + c * CELL_SIZE + CELL_SIZE / 2;
    text("C" + nf(c + 1, 2), x, gridStartY - 5);
  }

  // 恢复默认对齐
  textAlign(CENTER, CENTER);
}

/**
 * 在网格右侧绘制垂直色条图例，显示压力值与颜色的对应关系。
 */
void drawColorLegend() {
  float legendX = PADDING + COLS * CELL_SIZE + LEGEND_GAP;
  float legendY = PADDING;
  float legendH = ROWS * CELL_SIZE;

  // 绘制渐变色条
  int steps = (int) legendH;
  for (int i = 0; i < steps; i++) {
    // 从顶部(高压/红色)到底部(低压/绿色)
    float pressure = map(i, 0, steps - 1, PRESSURE_MAX, PRESSURE_MIN);
    color c = pressureToColor(pressure);
    stroke(c);
    line(legendX, legendY + i, legendX + LEGEND_WIDTH, legendY + i);
  }

  // 色条边框
  noFill();
  stroke(120);
  strokeWeight(1);
  rect(legendX, legendY, LEGEND_WIDTH, legendH);

  // 标注数值刻度
  fill(200);
  textSize(10);
  textAlign(LEFT, CENTER);
  int numTicks = 6;  // 0, 6, 12, 18, 24, 30
  for (int i = 0; i <= numTicks; i++) {
    float val = map(i, 0, numTicks, PRESSURE_MAX, PRESSURE_MIN);
    float y = map(i, 0, numTicks, legendY, legendY + legendH);

    // 刻度线
    stroke(200);
    strokeWeight(1);
    line(legendX + LEGEND_WIDTH, y, legendX + LEGEND_WIDTH + 4, y);

    // 数值标签
    noStroke();
    text(nf(val, 0, 0), legendX + LEGEND_WIDTH + 8, y);
  }

  // 图例标题
  textAlign(CENTER, BOTTOM);
  textSize(11);
  fill(200);
  text("Pressure", legendX + LEGEND_WIDTH / 2, legendY - 8);

  // 恢复
  textAlign(CENTER, CENTER);
}

/**
 * 在窗口底部绘制状态信息。
 */
void drawStatusBar() {
  float statusY = PADDING + ROWS * CELL_SIZE + 15;

  fill(150);
  textSize(10);
  textAlign(LEFT, TOP);

  String status = "Serial: " + (serialPort != null ? "Connected" : "Disconnected");
  status += "  |  Data FPS: " + nf(fps, 0, 1);
  status += "  |  Range: " + nf(PRESSURE_MIN, 0, 0) + " ~ " + nf(PRESSURE_MAX, 0, 0);

  text(status, PADDING, statusY);

  textAlign(CENTER, CENTER);
}

// ==================== Processing serialEvent ====================

/**
 * Processing 的串口事件回调（备用）。
 * 主要数据处理已在 draw() 中的 processSerialData() 完成，
 * 此回调作为额外的触发机制。
 */
void serialEvent(Serial port) {
  // 数据由 processSerialData() 在 draw() 中统一处理
  // 此处留空，避免重复处理
}

// ==================== 键盘交互 ====================

void keyPressed() {
  // 按 'R' 重置所有数据为无效状态
  if (key == 'r' || key == 'R') {
    initDataArrays();
    println("数据已重置");
  }

  // 按 'D' 填充演示数据（用于无串口时测试）
  if (key == 'd' || key == 'D') {
    fillDemoData();
    println("已填充演示数据");
  }

  // 按 'S' 保存当前帧为截图
  if (key == 's' || key == 'S') {
    saveFrame("heatmap_####.png");
    println("截图已保存");
  }
}

/**
 * 填充演示数据，用于在无串口连接时测试可视化效果。
 * 生成一个从中心向外扩散的高斯分布压力场。
 */
void fillDemoData() {
  float centerR = ROWS / 2.0;
  float centerC = COLS / 2.0;
  float sigma = 3.0;

  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      float dr = r - centerR;
      float dc = c - centerC;
      float dist2 = dr * dr + dc * dc;
      // 高斯分布 + 随机噪声
      pressureData[r][c] = PRESSURE_MAX * exp(-dist2 / (2 * sigma * sigma));
      pressureData[r][c] += random(-1, 1);
      pressureData[r][c] = constrain(pressureData[r][c], PRESSURE_MIN, PRESSURE_MAX);
    }
  }
}
