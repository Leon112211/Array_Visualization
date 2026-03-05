/***************************************************************
 * 项目：12x12 压阻阵列扫描系统
 * 硬件：AD5754R (3片菊链) + MCP23017 + ADG1206/ADG1419
 * 平台：Teensy 4.1
 * 版本：高速扫描优化版 (修复DEFAULT错误)
 ***************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <string.h>

// ==================== 系统配置常量 ====================
#define NUM_DACS 3           // DAC片数
#define NUM_DAC_CHANNELS 4   // 每片DAC通道数
#define NUM_COLUMNS 12       // 列数
#define NUM_ROWS 12          // 行数
#define TOTAL_CHANNELS (NUM_ROWS * NUM_COLUMNS)

// ==================== SPI 引脚定义 ====================
// AD5754R 使用 SPI0
#define MOSI_PIN 11
#define MISO_PIN 12
#define SCK_PIN  13
#define SYNC_PIN 37 
#define LDAC_PIN 31 
#define CLR_PIN  30 

// ==================== Teensy ADC 引脚 ====================
#define ADC_PIN 26           // 使用39号模拟输入引脚

// ==================== 电源控制引脚 ====================
const int vssPin = 40;     // 负电源使能
const int vddPin = 38;     // 正电源使能
const int switchPin = 35;  // 2.5V开关主控使能

// ==================== 矩阵切换引脚 ====================
const int EN_PIN_ADG1206 = 2;
const int ADDR_PINS_ADG1206[4] = {3, 4, 5, 6};
const int IN_PIN_ADG1419 = 7;

// ==================== MCP23017引脚分配 ====================
const int rowPins[NUM_ROWS] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
const int columnPins[NUM_COLUMNS] = {1, 2, 3, 4, 5, 6, 10, 11, 12, 13, 14, 15};

// ==================== 物理通道与逻辑行/列的映射 ====================
const int physicalToLogicalColumn[16] = {
  -1,  // 0: 无效
  7,   // 1 -> 列7
  8,   // 2 -> 列8
  9,   // 3 -> 列9
  10,  // 4 -> 列10
  11,  // 5 -> 列11
  12,  // 6 -> 列12
  -1, -1, -1,  // 7,8,9: 无效
  6,   // 10 -> 列6
  5,   // 11 -> 列5
  4,   // 12 -> 列4
  3,   // 13 -> 列3
  2,   // 14 -> 列2
  1    // 15 -> 列1
};

const int physicalToLogicalRow[NUM_ROWS] = {
  1,   // 物理行0 -> 逻辑行1
  4,   // 物理行1 -> 逻辑行4
  5,   // 物理行2 -> 逻辑行5
  8,   // 物理行3 -> 逻辑行8
  9,   // 物理行4 -> 逻辑行9
  12,  // 物理行5 -> 逻辑行12
  11,  // 物理行6 -> 逻辑行11
  10,  // 物理行7 -> 逻辑行10
  7,   // 物理行8 -> 逻辑行7
  6,   // 物理行9 -> 逻辑行6
  3,   // 物理行10 -> 逻辑行3
  2    // 物理行11 -> 逻辑行2
};

// ==================== 自定义扫描顺序 ====================
const int columnScanOrder_Physical[NUM_COLUMNS] = {15, 14, 13, 12, 11, 10, 1, 2, 3, 4, 5, 6};
const int rowScanOrder_Physical[NUM_ROWS] = {0, 11, 10, 1, 2, 9, 8, 3, 4, 7, 6, 5};

// ==================== AD5754R 寄存器定义 ====================
#define AD5754R_REG_DAC           0x00
#define AD5754R_REG_RANGE_SELECT  0x01
#define AD5754R_REG_POWER_CONTROL 0x02
#define AD5754R_REG_CONTROL       0x03

#define AD5754R_DAC_A 0x00
#define AD5754R_DAC_B 0x01
#define AD5754R_DAC_C 0x02
#define AD5754R_DAC_D 0x03
#define AD5754R_DAC_ALL 0x04

#define AD5754R_BIPOLAR_10_8_RANGE 0x05

static const uint8_t NOP_FRAME[3] = {0x18, 0x00, 0x00};

// ==================== 高速扫描配置 ====================
// 扫描时序配置 - 优化为1秒内读完144通道
const unsigned long CHANNEL_SETTLE_TIME_US = 500;  // 0.5ms每通道稳定时间 (6ms * 144 = 864ms < 1秒)
const unsigned long DISPLAY_INTERVAL = 100;        // 0.1秒显示一次

// ADC 配置 - 优化高速读取
#define ADC_READING_SAMPLES 4          // 4次采样平均（平衡速度和精度）
#define ADC_SETTLE_TIME_US 100         // 稳定时间100微秒
#define ADC_SAMPLE_INTERVAL_US 20      // 采样间隔20微秒

// Teensy ADC 配置
#define ADC_REFERENCE_VOLTAGE 3.3f     // Teensy 参考电压
#define ADC_RESOLUTION 10               // 10位ADC (0-1023)

// ==================== 全局对象 ====================
Adafruit_MCP23X17 mcp;
#define MCP_I2C_ADDR 0x20
TwoWire &i2cBus = Wire2;

// ==================== 数据结构 ====================
float channelVoltages[NUM_ROWS][NUM_COLUMNS] = {{0}};  // 电压矩阵 [逻辑行-1][逻辑列-1]
uint16_t channelRaw[NUM_ROWS][NUM_COLUMNS] = {{0}};    // 原始ADC值矩阵 [逻辑行-1][逻辑列-1]

// ==================== SPI互斥锁 ====================
class SPIMutex {
private:
  static bool ad5754r_in_use;

public:
  static bool acquireAD5754R() {
    if (ad5754r_in_use) return false;
    ad5754r_in_use = true;
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));
    return true;
  }

  static void releaseAD5754R() {
    SPI.endTransaction();
    ad5754r_in_use = false;
  }
};

bool SPIMutex::ad5754r_in_use = false;

// ==================== 扫描统计变量 ====================
unsigned long totalScans = 0;           // 完整扫描次数
unsigned long validReadings = 0;         // 有效读数次数
unsigned long lastDisplayTime = 0;
unsigned long lastReadings = 0;

// ==================== AD5754R 函数 ====================

static inline uint8_t make_cmd_byte(bool rw, uint8_t reg, uint8_t addr) {
  return (rw ? 0x80 : 0x00) | ((reg & 0x07) << 3) | (addr & 0x07);
}

void spiWriteChain(uint8_t frames[NUM_DACS][3]) {
  if (!SPIMutex::acquireAD5754R()) {
    Serial.println(F("[错误] 无法获取 AD5754R SPI 总线"));
    return;
  }

  digitalWrite(SYNC_PIN, LOW);
  delayMicroseconds(2);
  
  for (int i = NUM_DACS - 1; i >= 0; --i) {
    SPI.transfer(frames[i][0]);
    SPI.transfer(frames[i][1]);
    SPI.transfer(frames[i][2]);
  }
  
  digitalWrite(SYNC_PIN, HIGH);
  SPIMutex::releaseAD5754R();
}

static inline void pulseLDAC() {
  delayMicroseconds(1);
  digitalWrite(LDAC_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(LDAC_PIN, HIGH);
  delayMicroseconds(1);
}

static inline uint16_t volt2code(float v) {
  if (v < -10.8f) v = -10.8f;
  if (v > 10.8f) v = 10.8f;
  float t = (v + 10.8f) / 21.6f;
  uint32_t code = (uint32_t)lroundf(t * 65535.0f);
  return (code > 65535) ? 65535 : (uint16_t)code;
}

bool writePhysicalDAC(uint8_t chip, uint8_t ch, uint16_t code) {
  if (chip >= NUM_DACS || ch >= 4) return false;
  
  uint8_t frames[NUM_DACS][3];
  for (int i = 0; i < NUM_DACS; i++) {
    memcpy(frames[i], NOP_FRAME, 3);
  }

  uint8_t cmd = make_cmd_byte(false, AD5754R_REG_DAC, ch);
  frames[chip][0] = cmd;
  frames[chip][1] = (code >> 8) & 0xFF;
  frames[chip][2] = code & 0xFF;

  spiWriteChain(frames);
  return true;
}

// ==================== 修正版ADC初始化函数 ====================

void initTeensyADC() {
  Serial.println(F("\n=== 初始化 Teensy ADC (高速模式) ==="));
  
  pinMode(ADC_PIN, INPUT);
  
  // Teensy 4.1 的ADC配置
  analogReadResolution(ADC_RESOLUTION);  // 设置分辨率
  // Teensy 自动使用3.3V参考，不需要设置DEFAULT
  
  // 预读取几次来稳定ADC
  Serial.print(F("ADC预热读数: "));
  for (int i = 0; i < 5; i++) {
    int val = analogRead(ADC_PIN);
    Serial.print(val);
    Serial.print(" ");
    delayMicroseconds(500);
  }
  Serial.println();
  
  Serial.print(F("ADC初始化完成，分辨率: "));
  Serial.print(ADC_RESOLUTION);
  Serial.print(F("位，参考电压: "));
  Serial.print(ADC_REFERENCE_VOLTAGE);
  Serial.println(F("V"));
}

// 快速单次读取（用于丢弃）
inline uint16_t readADCFast() {
  return analogRead(ADC_PIN);
}

// 优化的平均读取（带采样间隔）
inline uint16_t readADCAveraged() {
  uint32_t sum = 0;
  
  // 快速连续采样
  for (int i = 0; i < ADC_READING_SAMPLES; i++) {
    sum += analogRead(ADC_PIN);
    // 极短延时，给ADC采样电容充电时间
    delayMicroseconds(ADC_SAMPLE_INTERVAL_US);
  }
  
  return sum / ADC_READING_SAMPLES;
}

// 电压转换
inline float rawToVoltage(uint16_t raw) {
  return (raw / 1023.0f) * ADC_REFERENCE_VOLTAGE;
}

// ==================== AD5754R 初始化 ====================

bool initAD5754R() {
  Serial.println(F("\n=== 初始化 AD5754R ==="));
  
  uint8_t frames[NUM_DACS][3];

  // 设置量程为±10.8V
  Serial.println(F("设置量程为 ±10.8V..."));
  uint8_t cmd = make_cmd_byte(false, AD5754R_REG_RANGE_SELECT, AD5754R_DAC_ALL);
  for (int i = 0; i < NUM_DACS; i++) {
    frames[i][0] = cmd;
    frames[i][1] = 0x00;
    frames[i][2] = AD5754R_BIPOLAR_10_8_RANGE;
  }
  spiWriteChain(frames);
  pulseLDAC();
  delay(10);

  // 上电所有通道
  Serial.println(F("上电所有DAC通道..."));
  cmd = make_cmd_byte(false, AD5754R_REG_POWER_CONTROL, 0x00);
  for (int i = 0; i < NUM_DACS; i++) {
    frames[i][0] = cmd;
    frames[i][1] = 0x00;
    frames[i][2] = 0x0F;
  }
  spiWriteChain(frames);
  delay(10);

  Serial.println(F("AD5754R初始化完成\n"));
  return true;
}

// 配置所有DAC输出电压
void setupAllDACs() {
  Serial.println(F("\n=== 配置DAC输出电压 ==="));
  
  // 设置所有DAC输出为1.0V
  float dacVoltage = 1.0f;
  
  // 芯片0
  writePhysicalDAC(0, 0, volt2code(dacVoltage));  // row3
  writePhysicalDAC(0, 1, volt2code(dacVoltage));  // row4
  writePhysicalDAC(0, 2, volt2code(dacVoltage));  // row2
  writePhysicalDAC(0, 3, volt2code(dacVoltage));  // row1
  
  // 芯片1
  writePhysicalDAC(1, 0, volt2code(dacVoltage));  // row7
  writePhysicalDAC(1, 1, volt2code(dacVoltage));  // row8
  writePhysicalDAC(1, 2, volt2code(dacVoltage));  // row6
  writePhysicalDAC(1, 3, volt2code(dacVoltage));  // row5

  // 芯片2
  writePhysicalDAC(2, 0, volt2code(dacVoltage));  // row11
  writePhysicalDAC(2, 1, volt2code(dacVoltage));  // row12
  writePhysicalDAC(2, 2, volt2code(dacVoltage));  // row10
  writePhysicalDAC(2, 3, volt2code(dacVoltage));  // row9
  
  pulseLDAC();
  delay(10);
  
  Serial.print(F("所有DAC设置为 "));
  Serial.print(dacVoltage);
  Serial.println(F("V\n"));
  
  // 输出映射关系供参考
  Serial.println(F("DAC映射关系:"));
  Serial.println(F("芯片0: 通道3(行1), 通道2(行2), 通道0(行3), 通道1(行4)"));
  Serial.println(F("芯片1: 通道3(行5), 通道2(行6), 通道0(行7), 通道1(行8)"));
  Serial.println(F("芯片2: 通道3(行9), 通道2(行10), 通道0(行11), 通道1(行12)"));
}

// ==================== MCP23017 控制 ====================

bool initMCP23017() {
  Serial.println(F("\n=== 初始化 MCP23017 ==="));
  
  i2cBus.setSDA(25);
  i2cBus.setSCL(24);
  i2cBus.begin();
  i2cBus.setClock(400000);

  if (!mcp.begin_I2C(MCP_I2C_ADDR, &i2cBus)) {
    Serial.println(F("[错误] 未找到MCP23017"));
    return false;
  }

  for (int p = 0; p < 16; p++) {
    mcp.pinMode(p, OUTPUT);
    mcp.digitalWrite(p, HIGH);
  }

  Serial.println(F("MCP23017初始化完成"));
  return true;
}

void setRowState(int physicalRow, bool turnOff) {
  if (physicalRow < 0 || physicalRow >= NUM_ROWS) return;
  mcp.digitalWrite(rowPins[physicalRow], turnOff ? HIGH : LOW);
}

void setActiveRow(int physicalRow) {
  for (int r = 0; r < NUM_ROWS; r++) {
    setRowState(r, true);
  }
  if (physicalRow >= 0 && physicalRow < NUM_ROWS) {
    setRowState(physicalRow, false);
  }
}

// ==================== ADG1206 控制 ====================

void setColumnSwitch(int physicalColumn) {
  if (physicalColumn < 1 || physicalColumn > 15) {
    digitalWrite(EN_PIN_ADG1206, LOW);
    return;
  }

  digitalWrite(EN_PIN_ADG1206, HIGH);
  int address = physicalColumn - 1;
  for (int i = 0; i < 4; i++) {
    digitalWrite(ADDR_PINS_ADG1206[i], (address >> i) & 1);
  }
}

// ==================== 电源控制 ====================

void powerOn() {
  Serial.println(F("\n=== 启动电源 ==="));
  
  pinMode(vssPin, OUTPUT);
  pinMode(vddPin, OUTPUT);
  pinMode(switchPin, OUTPUT);
  
  digitalWrite(vssPin, LOW);
  digitalWrite(vddPin, LOW);
  digitalWrite(switchPin, LOW);

  Serial.println(F("开启负电源..."));
  digitalWrite(vssPin, HIGH);
  delay(1000);

  Serial.println(F("开启正电源..."));
  digitalWrite(vddPin, HIGH);
  delay(1000);

  Serial.println(F("开启2.5V参考..."));
  digitalWrite(switchPin, HIGH);
  delay(500);
  
  Serial.println(F("电源启动完成\n"));
}

// ==================== 高速扫描函数 ====================

void initHighSpeedScan() {
  Serial.println(F("\n=== 初始化高速扫描模式 ==="));
  
  // 预充电所有行（减少建立时间）
  Serial.println(F("预充电所有行..."));
  for (int r = 0; r < NUM_ROWS; r++) {
    setRowState(r, false);  // 短暂导通
    delayMicroseconds(100);
  }
  
  // 设置初始位置
  setActiveRow(rowScanOrder_Physical[0]);
  setColumnSwitch(columnScanOrder_Physical[0]);
  digitalWrite(IN_PIN_ADG1419, HIGH);
  
  // 清空数据数组
  memset(channelVoltages, 0, sizeof(channelVoltages));
  memset(channelRaw, 0, sizeof(channelRaw));
  
  // 计算理论扫描时间
  float theoreticalTime = (NUM_ROWS * NUM_COLUMNS * CHANNEL_SETTLE_TIME_US) / 1000.0f;
  
  Serial.print(F("每通道稳定时间: "));
  Serial.print(CHANNEL_SETTLE_TIME_US);
  Serial.println(F(" μs"));
  Serial.print(F("理论完整扫描时间: "));
  Serial.print(theoreticalTime);
  Serial.println(F(" ms"));
  Serial.print(F("理论扫描速率: "));
  Serial.print(1000000.0f / CHANNEL_SETTLE_TIME_US);
  Serial.println(F(" 通道/秒"));
  
  // 打印扫描顺序
  Serial.println(F("\n扫描顺序(物理 -> 逻辑):"));
  Serial.print(F("列顺序: "));
  for (int i = 0; i < NUM_COLUMNS; i++) {
    int phys = columnScanOrder_Physical[i];
    int log = physicalToLogicalColumn[phys];
    Serial.print(phys);
    Serial.print("(列");
    Serial.print(log);
    Serial.print(")");
    if (i < NUM_COLUMNS - 1) Serial.print(", ");
  }
  Serial.println();
  
  Serial.print(F("行顺序: "));
  for (int i = 0; i < NUM_ROWS; i++) {
    int phys = rowScanOrder_Physical[i];
    int log = physicalToLogicalRow[phys];
    Serial.print(phys);
    Serial.print("(行");
    Serial.print(log);
    Serial.print(")");
    if (i < NUM_ROWS - 1) Serial.print(", ");
  }
  Serial.println();
  
  Serial.println(F("高速扫描模式初始化完成\n"));
}

// 一次扫描所有144个通道
void scanAllChannels() {
  unsigned long scanStart = micros();
  
  for (int r = 0; r < NUM_ROWS; r++) {
    // 设置行
    int physicalRow = rowScanOrder_Physical[r];
    setActiveRow(physicalRow);
    
    for (int c = 0; c < NUM_COLUMNS; c++) {
      // 设置列
      int physicalCol = columnScanOrder_Physical[c];
      setColumnSwitch(physicalCol);
      digitalWrite(IN_PIN_ADG1419, HIGH);
      
      // 等待通道稳定
      delayMicroseconds(CHANNEL_SETTLE_TIME_US);
      
      // 丢弃第一次读数（开关瞬态）
      (void)readADCFast();
      delayMicroseconds(ADC_SETTLE_TIME_US);
      
      // 读取并平均
      uint16_t raw = readADCAveraged();
      float voltage = rawToVoltage(raw);
      
      // 转换为逻辑行列并存储
      int logicalRow = physicalToLogicalRow[physicalRow];
      int logicalCol = physicalToLogicalColumn[physicalCol];
      
      channelRaw[logicalRow - 1][logicalCol - 1] = raw;
      channelVoltages[logicalRow - 1][logicalCol - 1] = voltage;
    }
  }
  
  unsigned long scanTime = micros() - scanStart;
  totalScans++;
  validReadings += NUM_ROWS * NUM_COLUMNS;
  
  // 每10次扫描打印一次性能数据
  static int scanCount = 0;
  if (++scanCount >= 10) {
    Serial.print(F("一次完整扫描时间: "));
    Serial.print(scanTime / 1000.0f);
    Serial.print(F(" ms ("));
    Serial.print((NUM_ROWS * NUM_COLUMNS * 1000000.0f) / scanTime);
    Serial.println(F(" 通道/秒)"));
    scanCount = 0;
  }
}

// ==================== 显示函数 ====================

void printHeader() {
  Serial.println();
  Serial.print("时间(s)\t");
  for (int c = 1; c <= NUM_COLUMNS; c++) {
    Serial.print("C");
    if (c < 10) Serial.print("0");
    Serial.print(c);
    Serial.print("\t");
  }
  Serial.println();
  
  Serial.print("-------\t");
  for (int c = 0; c < NUM_COLUMNS; c++) {
    Serial.print("----\t");
  }
  Serial.println();
}

void printRow(int logicalRow) {
  Serial.print("R");
  if (logicalRow < 10) Serial.print("0");
  Serial.print(logicalRow);
  Serial.print("\t");
  
  for (int c = 1; c <= NUM_COLUMNS; c++) {
    char buffer[8];
    dtostrf(channelVoltages[logicalRow - 1][c - 1], 5, 2, buffer);
    Serial.print(buffer);
    Serial.print("\t");
  }
  Serial.println();
}

void printAllData() {
  Serial.print("\n=== 12x12 矩阵 @ ");
  Serial.print(millis() / 1000);
  Serial.print("s ===");
  Serial.print(" 扫描#"); Serial.print(totalScans);
  Serial.print(" 速率:"); 
  Serial.print((validReadings - lastReadings) / (DISPLAY_INTERVAL / 1000.0f));
  Serial.println(" ch/s");
  
  for (int r = 1; r <= NUM_ROWS; r++) {
    printRow(r);
  }
  Serial.println("========================\n");
}

// ==================== 验证函数 ====================

void verifySystem() {
  Serial.println(F("\n=== 系统验证 ==="));
  
  // 关闭所有行
  setActiveRow(-1);
  
  // 测试每个列
  for (int c = 0; c < NUM_COLUMNS; c++) {
    int physicalCol = columnPins[c];
    setColumnSwitch(physicalCol);
    delay(10);  // 缩短验证时间
    
    // 快速读取
    uint32_t sum = 0;
    int samples = 10;
    for (int i = 0; i < samples; i++) {
      sum += analogRead(ADC_PIN);
      delayMicroseconds(100);
    }
    float avg = (sum / samples) / 1023.0f * ADC_REFERENCE_VOLTAGE;
    
    int logicalCol = physicalToLogicalColumn[physicalCol];
    Serial.print("列");
    Serial.print(physicalCol);
    Serial.print("(逻辑");
    Serial.print(logicalCol);
    Serial.print("): ");
    Serial.print(avg, 3);
    Serial.println("V");
  }
  
  Serial.println(F("验证完成\n"));
}

// ==================== setup() ====================

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  Serial.println(F("\n========================================="));
  Serial.println(F("12x12 压阻阵列 - 高速扫描优化版"));
  Serial.println(F("========================================="));
  
  // 初始化引脚
  pinMode(SYNC_PIN, OUTPUT);
  pinMode(LDAC_PIN, OUTPUT);
  pinMode(CLR_PIN, OUTPUT);
  pinMode(EN_PIN_ADG1206, OUTPUT);
  pinMode(IN_PIN_ADG1419, OUTPUT);
  
  for (int i = 0; i < 4; i++) {
    pinMode(ADDR_PINS_ADG1206[i], OUTPUT);
  }
  
  digitalWrite(SYNC_PIN, HIGH);
  digitalWrite(LDAC_PIN, HIGH);
  digitalWrite(CLR_PIN, HIGH);
  digitalWrite(EN_PIN_ADG1206, LOW);
  digitalWrite(IN_PIN_ADG1419, LOW);
  
  // 启动SPI0
  SPI.begin();
  SPI.setMOSI(MOSI_PIN);
  SPI.setMISO(MISO_PIN);
  SPI.setSCK(SCK_PIN);
  
  // 启动电源
  powerOn();
  
  // 初始化MCP23017
  if (!initMCP23017()) {
    Serial.println(F("[致命错误] MCP23017初始化失败"));
    while (1) delay(1000);
  }
  
  // 初始化Teensy ADC
  initTeensyADC();
  
  // 初始化AD5754R
  if (!initAD5754R()) {
    Serial.println(F("[致命错误] AD5754R初始化失败"));
    while (1) delay(1000);
  }
  
  // 配置所有DAC电压
  setupAllDACs();
  
  // 快速验证系统
  verifySystem();
  
  // 初始化高速扫描
  initHighSpeedScan();
  
  // 打印表头
  printHeader();
  
  Serial.println(F("\n系统就绪，开始高速扫描 (目标: 144通道/秒)\n"));
}

// ==================== loop() ====================

void loop() {
  static unsigned long lastScanStart = 0;
  
  // 每 CHANNEL_SETTLE_TIME_US 微秒扫描一次所有通道
  if (micros() - lastScanStart >= CHANNEL_SETTLE_TIME_US) {
    scanAllChannels();
    lastScanStart = micros();
  }
  
  // 每秒显示数据
  if (millis() - lastDisplayTime >= DISPLAY_INTERVAL) {
    printAllData();
    lastReadings = validReadings;
    lastDisplayTime = millis();
  }
  
  // 极短延时，防止看门狗
  delayMicroseconds(10);
}