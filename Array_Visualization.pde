/***************************************************************
 * 项目：12x12 压力传感阵列 - 3D 实时热力图可视化
 * 平台：Processing 4.x
 * 依赖：Processing Serial Library (内置，无需额外安装)
 *
 * 功能：
 *   - 通过串口接收 Arduino/Teensy 发送的 12x12 矩阵数据
 *   - 将压力数值实时转化为 3D 立体柱状图和表面网格
 *   - 支持鼠标拖拽旋转、滚轮缩放、方向键平移
 *   - 支持平滑动画缓动，显示状态信息与色条
 *
 * 数据协议：
 *   Arduino 端以 "R01\t值1\t值2\t...\t值12" 格式逐行输出
 *   每帧由 "===" 开头的行标记起始/结束
 ***************************************************************/

import processing.serial.*;
import java.awt.Toolkit;
import java.awt.Dimension;

// ==================== 系统配置 ====================
final int ROWS = 12;
final int COLS = 12;

// 压力数值范围：0 = 无压力，30 = 满载压力
final float PRESSURE_MIN = 0.0;
final float PRESSURE_MAX = 30.0;

// 电压 -> 压力 的线性映射参数
final float VOLTAGE_MIN = 0.0;
final float VOLTAGE_MAX = 3.3;

// 网格绘制参数
final int CELL_SIZE = 50;          // 每个方格的像素大小
final int PADDING = 60;            // 画布四周留白
final int LEGEND_WIDTH = 30;       // 色条图例宽度
final int LEGEND_GAP = 30;         // 色条与网格的间距

// 串口配置
final int BAUD_RATE = 115200;
String SERIAL_PORT_NAME = "";

// ==================== 3D 视角控制 ====================
float rotationX = 1.0;
float rotationY = 0.0;
float zoom = 1.0;
float panX = 0;
float panY = 0;
final float PRESSURE_HEIGHT_MAX = 150.0; // 放大最大高度，使立体感更明显

// ==================== 自适应缩放 ====================
// 设计基准分辨率（所有布局基于此尺寸设计）
final int REF_WIDTH = 1000;
final int REF_HEIGHT = 800;
// 屏幕物理分辨率（在 settings() 中检测）
int screenW, screenH;
// 动态内容缩放因子（每帧根据窗口大小更新）
float contentScale = 1.0;

// ==================== 全局变量 ====================
Serial serialPort;

// 数据矩阵：存储当前帧的压力真实值
float[][] pressureData = new float[ROWS][COLS];
// 用于平滑动画的显示数据矩阵（缓动插值）
float[][] displayPressureData = new float[ROWS][COLS];

// 临时缓冲区：在接收完整帧期间暂存数据
float[][] bufferData = new float[ROWS][COLS];
boolean[] rowReceived = new boolean[ROWS];  // 标记哪些行已收到
boolean frameInProgress = false;            // 是否正在接收一帧数据

// 演示模式切换（0=高斯, 1=环形）
int demoMode = 0;
final int DEMO_MODE_COUNT = 2;

// 帧计数器（用于显示统计）
int frameCount = 0;
long lastFrameTime = 0;
float fps = 0;

// ==================== Processing 入口 ====================

void settings() {
  // 检测显示器物理分辨率
  Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
  screenW = screenSize.width;
  screenH = screenSize.height;

  // 初始窗口取屏幕短边的 60%，保持 5:4 比例
  int windowWidth  = (int)(min(screenW, screenH) * 0.60 * 1.25); // 5:4
  int windowHeight = (int)(min(screenW, screenH) * 0.60);
  size(windowWidth, windowHeight, P3D);  // 启用 3D 渲染
  noSmooth();
}

void setup() {
  surface.setTitle("12x12 Pressure Array 3D Visualization");
  surface.setResizable(true);

  // 将窗口居中到屏幕中央
  surface.setLocation((screenW - width) / 2, (screenH - height) / 2);

  println("屏幕分辨率: " + screenW + " x " + screenH);
  println("初始窗口: " + width + " x " + height);

  // 初始化数据
  initDataArrays();

  // 初始化串口
  initSerial();

  // 设置文本属性
  textAlign(CENTER, CENTER);
  textSize(12);

  println("=== 3D可视化系统就绪 ===");
  println("窗口大小: " + width + " x " + height);
  println("压力范围: " + PRESSURE_MIN + " ~ " + PRESSURE_MAX);
}

void draw() {
  background(30);

  // 根据当前窗口大小计算内容缩放因子（取宽高中较小的比例，保证内容不溢出）
  contentScale = min((float)width / REF_WIDTH, (float)height / REF_HEIGHT);

  // 处理所有待读取的串口数据
  processSerialData();

  // 平滑插值更新显示数据
  updateDisplayData();

  // ----- 3D 环境绘制 -----
  pushMatrix();

  // 视角变换：将 3D 场景居中于当前窗口，并叠加自适应缩放
  translate(width / 2.0 + panX, height / 2.0 + panY, -200 * contentScale);
  rotateX(rotationX);
  rotateY(rotationY);
  scale(zoom * contentScale);

  // 光照设置
  lights();
  directionalLight(255, 255, 255, 0.5, 0.5, -1);
  ambientLight(100, 100, 100);

  // 绘制基面网格
  drawBaseGrid();

  // 绘制 3D 柱体和表面网格
  drawHeatmap3D();

  // 绘制行列标签
  drawLabels3D();

  popMatrix();

  // ----- 2D UI 绘制 -----
  hint(DISABLE_DEPTH_TEST);
  camera();      // 重置相机
  noLights();    // 关闭光照

  drawColorLegend();
  drawStatusBar();
  drawInstructions();

  hint(ENABLE_DEPTH_TEST);
}

// ==================== 数据更新与插值 ====================

void updateDisplayData() {
  // 动画缓动：让显示的数据平滑过渡到目标真实数据
  float lerpFactor = 0.2; // 插值系数，调节过渡速度
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      float target = pressureData[r][c];
      if (Float.isNaN(target)) target = 0; // 无效数据当0处理

      if (Float.isNaN(displayPressureData[r][c])) {
        displayPressureData[r][c] = target;
      } else {
        displayPressureData[r][c] = lerp(displayPressureData[r][c], target, lerpFactor);
      }
    }
  }
}

void initDataArrays() {
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      pressureData[r][c] = Float.NaN;
      displayPressureData[r][c] = Float.NaN;
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
    portName = portList[portList.length - 1];
  }

  try {
    serialPort = new Serial(this, portName, BAUD_RATE);
    serialPort.bufferUntil('\n');
    println("已连接串口: " + portName + " @ " + BAUD_RATE + " baud");
  } catch (Exception e) {
    println("[错误] 无法打开串口 " + portName + ": " + e.getMessage());
    serialPort = null;
  }
}

// ==================== 串口数据处理 ====================

void processSerialData() {
  if (serialPort == null) return;

  while (serialPort.available() > 0) {
    String line = serialPort.readStringUntil('\n');
    if (line == null) break;

    line = line.trim();
    if (line.length() == 0) continue;

    if (line.startsWith("=== 12x12")) {
      frameInProgress = true;
      for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
          bufferData[r][c] = Float.NaN;
        }
        rowReceived[r] = false;
      }
      continue;
    }

    if (line.startsWith("====") && !line.contains("12x12")) {
      if (frameInProgress) {
        commitFrame();
        frameInProgress = false;
      }
      continue;
    }

    if (frameInProgress && line.startsWith("R")) {
      parseRowData(line);
    }
  }
}

void parseRowData(String line) {
  String[] parts = line.split("\t");
  if (parts.length < 2) return;

  String rowStr = parts[0].trim();
  if (rowStr.length() < 2) return;

  int rowIndex;
  try {
    rowIndex = Integer.parseInt(rowStr.substring(1)) - 1;
  } catch (NumberFormatException e) {
    return;
  }

  if (rowIndex < 0 || rowIndex >= ROWS) return;

  for (int c = 0; c < COLS && c + 1 < parts.length; c++) {
    String valStr = parts[c + 1].trim();
    try {
      float voltage = Float.parseFloat(valStr);
      float pressure = map(voltage, VOLTAGE_MIN, VOLTAGE_MAX, PRESSURE_MIN, PRESSURE_MAX);
      pressure = constrain(pressure, PRESSURE_MIN, PRESSURE_MAX);
      bufferData[rowIndex][c] = pressure;
    } catch (NumberFormatException e) {
      bufferData[rowIndex][c] = Float.NaN;
    }
  }

  rowReceived[rowIndex] = true;
}

void commitFrame() {
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      pressureData[r][c] = bufferData[r][c];
    }
  }

  frameCount++;
  long now = millis();
  if (now - lastFrameTime > 1000) {
    fps = frameCount * 1000.0 / (now - lastFrameTime);
    frameCount = 0;
    lastFrameTime = now;
  }
}

// ==================== 颜色映射 ====================

color pressureToColor(float pressure) {
  if (Float.isNaN(pressure)) {
    return color(255, 255, 255);
  }

  pressure = constrain(pressure, PRESSURE_MIN, PRESSURE_MAX);
  float t = map(pressure, PRESSURE_MIN, PRESSURE_MAX, 0.0, 1.0);

  float h = map(t, 0.0, 1.0, 120.0, 0.0);
  float s = map(t, 0.0, 1.0, 25.0, 100.0);
  float b = map(t, 0.0, 1.0, 100.0, 70.0);

  colorMode(HSB, 360, 100, 100);
  color c = color(h, s, b);
  colorMode(RGB, 255);

  return c;
}

// ==================== 3D 绘图函数 ====================

float getSafePressure(int r, int c) {
  float p = displayPressureData[r][c];
  return Float.isNaN(p) ? 0 : p;
}

// 获取 13×13 格点处的插值压力值（格点位于网格交叉点，通过相邻单元格均值计算）
float getGridPointPressure(int gr, int gc) {
  // gr: 0~ROWS, gc: 0~COLS — 格点索引
  // 收集该格点相邻的 1~4 个单元格的压力值取均值
  float sum = 0;
  int count = 0;
  // 格点 (gr, gc) 的相邻单元格为 (gr-1,gc-1), (gr-1,gc), (gr,gc-1), (gr,gc)
  for (int dr = -1; dr <= 0; dr++) {
    for (int dc = -1; dc <= 0; dc++) {
      int r = gr + dr;
      int c = gc + dc;
      if (r >= 0 && r < ROWS && c >= 0 && c < COLS) {
        sum += getSafePressure(r, c);
        count++;
      }
    }
  }
  return (count > 0) ? sum / count : 0;
}

// 格点坐标转换：格点索引 → 世界坐标
float gridPointX(int gc) { return (gc - COLS / 2.0) * CELL_SIZE; }
float gridPointY(int gr) { return (gr - ROWS / 2.0) * CELL_SIZE; }
float gridPointZ(float pressure) { return map(pressure, PRESSURE_MIN, PRESSURE_MAX, 0, -PRESSURE_HEIGHT_MAX); }

void drawBaseGrid() {
  // 绘制贴合曲面的 3D 网格线（13 横 + 13 纵）
  stroke(80);
  strokeWeight(1);
  noFill();

  // 横向网格线 (13 条, gr = 0 ~ ROWS)
  for (int gr = 0; gr <= ROWS; gr++) {
    beginShape();
    for (int gc = 0; gc <= COLS; gc++) {
      float p = getGridPointPressure(gr, gc);
      vertex(gridPointX(gc), gridPointY(gr), gridPointZ(p));
    }
    endShape();
  }

  // 纵向网格线 (13 条, gc = 0 ~ COLS)
  for (int gc = 0; gc <= COLS; gc++) {
    beginShape();
    for (int gr = 0; gr <= ROWS; gr++) {
      float p = getGridPointPressure(gr, gc);
      vertex(gridPointX(gc), gridPointY(gr), gridPointZ(p));
    }
    endShape();
  }
}

void drawHeatmap3D() {
  // 逐单元格绘制：每格统一颜色，四角使用格点Z高度保持曲面形状
  noStroke();
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      // 该单元格的统一颜色
      fill(pressureToColor(getSafePressure(r, c)));

      // 四角格点的 Z 高度（保持曲面凹陷）
      float z00 = gridPointZ(getGridPointPressure(r,     c    ));
      float z01 = gridPointZ(getGridPointPressure(r,     c + 1));
      float z10 = gridPointZ(getGridPointPressure(r + 1, c    ));
      float z11 = gridPointZ(getGridPointPressure(r + 1, c + 1));

      float x0 = gridPointX(c);
      float x1 = gridPointX(c + 1);
      float y0 = gridPointY(r);
      float y1 = gridPointY(r + 1);

      // 两个三角形拼成一个单元格
      beginShape(TRIANGLES);
      vertex(x0, y0, z00);
      vertex(x1, y0, z01);
      vertex(x0, y1, z10);

      vertex(x1, y0, z01);
      vertex(x1, y1, z11);
      vertex(x0, y1, z10);
      endShape();
    }
  }
}

void drawLabels3D() {
  fill(200);
  textSize(14);
  textAlign(CENTER, CENTER);

  float w = COLS * CELL_SIZE;
  float h = ROWS * CELL_SIZE;

  // 绘制行标签 (R01 ~ R12)
  for (int r = 0; r < ROWS; r++) {
    float y = (r - ROWS/2.0 + 0.5) * CELL_SIZE;
    pushMatrix();
    translate(-w/2 - 25, y, 0);
    text("R" + nf(r + 1, 2), 0, 0);
    popMatrix();
  }

  // 绘制列标签 (C01 ~ C12)
  for (int c = 0; c < COLS; c++) {
    float x = (c - COLS/2.0 + 0.5) * CELL_SIZE;
    pushMatrix();
    translate(x, -h/2 - 25, 0);
    text("C" + nf(c + 1, 2), 0, 0);
    popMatrix();
  }
}

// ==================== 2D UI 函数 ====================

void drawColorLegend() {
  float s = contentScale; // 缩放因子
  float lw = LEGEND_WIDTH * s;
  float pad = PADDING * s;
  float legendX = width - lw - pad;
  float legendH = 300 * s;
  float legendY = (height - legendH) / 2.0; // 垂直居中

  // 绘制渐变色条
  int steps = max((int) legendH, 1);
  for (int i = 0; i < steps; i++) {
    float pressure = map(i, 0, steps - 1, PRESSURE_MAX, PRESSURE_MIN);
    color c = pressureToColor(pressure);
    stroke(c);
    line(legendX, legendY + i, legendX + lw, legendY + i);
  }

  // 色条边框
  noFill();
  stroke(120);
  strokeWeight(1);
  rect(legendX, legendY, lw, legendH);

  // 数值刻度
  fill(200);
  textSize(12 * s);
  textAlign(LEFT, CENTER);
  int numTicks = 6;
  for (int i = 0; i <= numTicks; i++) {
    float val = map(i, 0, numTicks, PRESSURE_MAX, PRESSURE_MIN);
    float y = map(i, 0, numTicks, legendY, legendY + legendH);

    stroke(200);
    line(legendX + lw, y, legendX + lw + 4 * s, y);

    noStroke();
    text(nf(val, 0, 0), legendX + lw + 8 * s, y);
  }

  // 标题
  textAlign(CENTER, BOTTOM);
  textSize(12 * s);
  fill(200);
  text("Pressure", legendX + lw / 2, legendY - 8 * s);

  textAlign(CENTER, CENTER);
}

void drawStatusBar() {
  float s = contentScale;
  fill(150);
  textSize(12 * s);
  textAlign(LEFT, BOTTOM);

  String status = "Serial: " + (serialPort != null ? "Connected" : "Disconnected");
  status += "  |  Data FPS: " + nf(fps, 0, 1);
  status += "  |  Range: " + nf(PRESSURE_MIN, 0, 0) + " ~ " + nf(PRESSURE_MAX, 0, 0);
  status += "  |  Window: " + width + "x" + height;

  text(status, 20 * s, height - 20 * s);
}

void drawInstructions() {
  float s = contentScale;
  fill(180);
  textSize(12 * s);
  textAlign(LEFT, TOP);
  String ins = "CONTROLS:\n";
  ins += "[Left Click Drag] Rotate\n";
  ins += "[Mouse Wheel] Zoom\n";
  ins += "[Arrow Keys] Pan\n";
  ins += "[D] Fill Demo Data\n";
  ins += "[R] Reset View & Data\n";
  ins += "[S] Screenshot";
  text(ins, 20 * s, 20 * s);
}

// ==================== 鼠标交互 ====================

void mouseDragged() {
  if (mouseButton == LEFT) {
    rotationX += (mouseY - pmouseY) * -0.01;
    rotationY += (mouseX - pmouseX) * 0.01;
  }
}

void mouseWheel(MouseEvent event) {
  zoom *= pow(1.1, -event.getCount());
}

// ==================== 键盘交互 ====================

void keyPressed() {
  // 方向键控制平移
  if (key == CODED) {
    if (keyCode == UP) panY += 20;
    if (keyCode == DOWN) panY -= 20;
    if (keyCode == LEFT) panX += 20;
    if (keyCode == RIGHT) panX -= 20;
  } else {
    // 按 'R' 重置
    if (key == 'r' || key == 'R') {
      initDataArrays();
      panX = 0; panY = 0; zoom = 1.0; rotationX = 1.0; rotationY = 0.0;
      println("数据和视角已重置");
    }
    // 按 'D' 生成演示数据
    if (key == 'd' || key == 'D') {
      fillDemoData();
      println("已填充演示数据");
    }
    // 按 'S' 截图
    if (key == 's' || key == 'S') {
      saveFrame("heatmap_####.png");
      println("截图已保存");
    }
  }
}

void fillDemoData() {
  float centerR = ROWS / 2.0;
  float centerC = COLS / 2.0;

  if (demoMode == 0) {
    // 高斯分布：中心最大，向外衰减
    float sigma = 3.0;
    for (int r = 0; r < ROWS; r++) {
      for (int c = 0; c < COLS; c++) {
        float dr = r - centerR;
        float dc = c - centerC;
        float dist2 = dr * dr + dc * dc;
        pressureData[r][c] = PRESSURE_MAX * exp(-dist2 / (2 * sigma * sigma));
        pressureData[r][c] += random(-1, 1);
        pressureData[r][c] = constrain(pressureData[r][c], PRESSURE_MIN, PRESSURE_MAX);
      }
    }
  } else if (demoMode == 1) {
    // 环形分布：压力集中在半径 ringR 附近的环带，中心和边缘较低
    float ringR = 3.5;    // 环的半径（单元格数）
    float ringW = 1.8;    // 环的宽度（控制衰减）
    for (int r = 0; r < ROWS; r++) {
      for (int c = 0; c < COLS; c++) {
        float dr = r - centerR;
        float dc = c - centerC;
        float dist = sqrt(dr * dr + dc * dc);
        float deviation = dist - ringR;
        pressureData[r][c] = PRESSURE_MAX * exp(-(deviation * deviation) / (2 * ringW * ringW));
        pressureData[r][c] += random(-1, 1);
        pressureData[r][c] = constrain(pressureData[r][c], PRESSURE_MIN, PRESSURE_MAX);
      }
    }
  }

  // 切换到下一个模式
  demoMode = (demoMode + 1) % DEMO_MODE_COUNT;
}
