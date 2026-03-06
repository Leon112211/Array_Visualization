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

// 帧计数器（用于显示统计）
int frameCount = 0;
long lastFrameTime = 0;
float fps = 0;

// ==================== Processing 入口 ====================

void settings() {
  int windowWidth = 1000;
  int windowHeight = 800;
  size(windowWidth, windowHeight, P3D);  // 启用 3D 渲染
  noSmooth();
}

void setup() {
  surface.setTitle("12x12 Pressure Array 3D Visualization");

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

  // 处理所有待读取的串口数据
  processSerialData();

  // 平滑插值更新显示数据
  updateDisplayData();

  // ----- 3D 环境绘制 -----
  pushMatrix();

  // 视角变换
  translate(width / 2.0 + panX, height / 2.0 + panY, -200);
  rotateX(rotationX);
  rotateY(rotationY);
  scale(zoom);

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

void drawBaseGrid() {
  pushMatrix();
  float w = COLS * CELL_SIZE;
  float h = ROWS * CELL_SIZE;

  translate(0, 0, -1); // 稍微下移避免与柱体底部产生 Z-fighting

  // 绘制深色底板
  fill(45);
  noStroke();
  rectMode(CENTER);
  rect(0, 0, w + 20, h + 20);
  rectMode(CORNER);

  // 绘制网格线
  stroke(80);
  strokeWeight(1);
  for (int r = 0; r <= ROWS; r++) {
    float y = (r - ROWS/2.0) * CELL_SIZE;
    line(-w/2, y, 0, w/2, y, 0);
  }
  for (int c = 0; c <= COLS; c++) {
    float x = (c - COLS/2.0) * CELL_SIZE;
    line(x, -h/2, 0, x, h/2, 0);
  }
  popMatrix();
}

void drawHeatmap3D() {
  // 1. 绘制柱体与数值
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      drawCell3D(r, c, displayPressureData[r][c]);
    }
  }

  // 2. 绘制表面网格（增强效果）
  noFill();
  stroke(255, 120);
  strokeWeight(1.5);
  beginShape(LINES);
  for (int r = 0; r < ROWS; r++) {
    for (int c = 0; c < COLS; c++) {
      float x1 = (c - COLS/2.0 + 0.5) * CELL_SIZE;
      float y1 = (r - ROWS/2.0 + 0.5) * CELL_SIZE;
      float p1 = displayPressureData[r][c];
      float h1 = map(p1, PRESSURE_MIN, PRESSURE_MAX, 0, PRESSURE_HEIGHT_MAX);

      // 连接右边节点
      if (c < COLS - 1) {
        float x2 = (c + 1 - COLS/2.0 + 0.5) * CELL_SIZE;
        float p2 = displayPressureData[r][c+1];
        float h2 = map(p2, PRESSURE_MIN, PRESSURE_MAX, 0, PRESSURE_HEIGHT_MAX);
        vertex(x1, y1, h1);
        vertex(x2, y1, h2);
      }
      // 连接下边节点
      if (r < ROWS - 1) {
        float y2 = (r + 1 - ROWS/2.0 + 0.5) * CELL_SIZE;
        float p2 = displayPressureData[r+1][c];
        float h2 = map(p2, PRESSURE_MIN, PRESSURE_MAX, 0, PRESSURE_HEIGHT_MAX);
        vertex(x1, y1, h1);
        vertex(x1, y2, h2);
      }
    }
  }
  endShape();
}

void drawCell3D(int r, int c, float pressure) {
  if (Float.isNaN(pressure) || pressure <= 0.1) return; // 忽略微小压力和无效数据

  float x = (c - COLS/2.0 + 0.5) * CELL_SIZE;
  float y = (r - ROWS/2.0 + 0.5) * CELL_SIZE;
  float boxH = map(pressure, PRESSURE_MIN, PRESSURE_MAX, 0, PRESSURE_HEIGHT_MAX);

  color cellColor = pressureToColor(pressure);

  pushMatrix();
  translate(x, y, boxH / 2.0); // box的绘制锚点在中心

  fill(cellColor);
  noStroke(); // 无边框避免视觉杂乱
  box(CELL_SIZE - 4, CELL_SIZE - 4, boxH);

  popMatrix();

  // 在柱体上方绘制数值标签
  if (pressure > 2.0) { // 压力大于一定值才显示，避免拥挤
    pushMatrix();
    translate(x, y, boxH + 2);
    fill(255);
    textSize(11);
    textAlign(CENTER, CENTER);
    text(nf(pressure, 0, 1), 0, 0);
    popMatrix();
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
  float legendX = width - LEGEND_WIDTH - PADDING;
  float legendH = 300;
  float legendY = (height - legendH) / 2.0; // 垂直居中

  // 绘制渐变色条
  int steps = (int) legendH;
  for (int i = 0; i < steps; i++) {
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

  // 数值刻度
  fill(200);
  textSize(12);
  textAlign(LEFT, CENTER);
  int numTicks = 6;
  for (int i = 0; i <= numTicks; i++) {
    float val = map(i, 0, numTicks, PRESSURE_MAX, PRESSURE_MIN);
    float y = map(i, 0, numTicks, legendY, legendY + legendH);

    stroke(200);
    line(legendX + LEGEND_WIDTH, y, legendX + LEGEND_WIDTH + 4, y);

    noStroke();
    text(nf(val, 0, 0), legendX + LEGEND_WIDTH + 8, y);
  }

  // 标题
  textAlign(CENTER, BOTTOM);
  textSize(12);
  fill(200);
  text("Pressure", legendX + LEGEND_WIDTH / 2, legendY - 8);

  textAlign(CENTER, CENTER);
}

void drawStatusBar() {
  fill(150);
  textSize(12);
  textAlign(LEFT, BOTTOM);

  String status = "Serial: " + (serialPort != null ? "Connected" : "Disconnected");
  status += "  |  Data FPS: " + nf(fps, 0, 1);
  status += "  |  Range: " + nf(PRESSURE_MIN, 0, 0) + " ~ " + nf(PRESSURE_MAX, 0, 0);

  text(status, 20, height - 20);
}

void drawInstructions() {
  fill(180);
  textSize(12);
  textAlign(LEFT, TOP);
  String ins = "CONTROLS:\n";
  ins += "[Left Click Drag] Rotate\n";
  ins += "[Mouse Wheel] Zoom\n";
  ins += "[Arrow Keys] Pan\n";
  ins += "[D] Fill Demo Data\n";
  ins += "[R] Reset View & Data\n";
  ins += "[S] Screenshot";
  text(ins, 20, 20);
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
}
