# STM32H723 SEGGER RTT 移植完整指南

## 移植状态 ✅

### 已完成的文件
- ✅ `Middlewares/SEGGER_RTT/` - RTT源代码
- ✅ `Middlewares/Middlewares.cmake` - CMake配置
- ✅ `Utilities/openocd-cmsisdap-stm32h7x.cfg` - OpenOCD RTT配置
- ✅ `Applications/05_System/rtt_test.h/cpp` - RTT测试代码
- ✅ `Applications/05_System/rtt_example.cpp` - RTT使用示例

## 如何使用

### 1. 编译项目
```bash
cd build
cmake ..
ninja
```

### 2. 启动RTT调试
```bash
# 终端1: 启动OpenOCD
cd Utilities
openocd -f openocd-cmsisdap-stm32h7x.cfg

# 终端2: 连接RTT终端
telnet localhost 9090
```

### 3. 在代码中使用RTT

#### 基础使用：
```cpp
#include "rtt_test.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    
    // 初始化RTT
    RTT_Init();
    
    // 运行测试
    RTT_Test_Basic();
    
    while(1) {
        RTT_INFO("Hello RTT! Time: %lu\n", HAL_GetTick());
        HAL_Delay(1000);
    }
}
```

#### 在FreeRTOS任务中：
```cpp
void MyTask(void *argument) {
    RTT_INFO("Task started: %s\n", pcTaskGetName(NULL));
    
    while(1) {
        RTT_INFO("Task running at tick %lu\n", xTaskGetTickCount());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

#### 调试宏使用：
```cpp
RTT_ERROR("Critical error occurred!\n");
RTT_WARN("Warning: Low battery voltage\n");
RTT_INFO("System initialized successfully\n");
RTT_DEBUG("Debug: Variable x = %d\n", x);
```

### 4. 性能测试
在代码中调用：
```cpp
RTT_Test_Performance();  // 测试RTT传输性能
RTT_Test_DataTypes();    // 测试各种数据类型输出
```

### 5. printf重定向 (可选)
如果要将标准printf重定向到RTT，在 `rtt_test.cpp` 中取消注释：
```cpp
#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{
    SEGGER_RTT_Write(RTT_CHANNEL_TERMINAL, ptr, len);
    return len;
}
#endif
```

然后就可以直接使用printf：
```cpp
printf("Hello from printf! Counter: %d\n", counter);
```

## RTT配置说明

### 缓冲区大小 (已优化为STM32H723)
- 上行缓冲区: 2048 bytes (Terminal输出)
- 下行缓冲区: 64 bytes (输入)
- Printf缓冲区: 128 bytes

### RTT通道
- 通道0: Terminal (端口9090)
- 通道1: Debug (可选)

### 内存范围
- 起始: 0x20000000 (DTCM)
- 结束: 0x24080000 (覆盖DTCM + AXI SRAM)

## 故障排除

### 1. 无法找到RTT控制块
- 确保程序中调用了 `RTT_Init()` 或任何RTT输出函数
- 检查内存范围是否正确
- 确认程序已正确加载到MCU

### 2. 无法连接端口9090
```bash
# 检查OpenOCD是否正常运行
ps aux | grep openocd

# 检查端口是否被占用
netstat -an | grep 9090

# 检查防火墙设置
sudo ufw allow 9090
```

### 3. RTT输出不完整
- 增加缓冲区大小
- 使用阻塞模式：
```cpp
SEGGER_RTT_SetFlagsUpBuffer(0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
```

### 4. 性能问题
- 减少输出频率
- 使用批量输出
- 避免在高频中断中使用RTT

## 验证清单

- [ ] 项目能正常编译
- [ ] OpenOCD能正常启动并找到RTT控制块
- [ ] 能通过telnet连接端口9090
- [ ] 能看到RTT初始化信息
- [ ] 基础测试通过
- [ ] 性能测试正常
- [ ] 在主循环中能正常输出
- [ ] 在FreeRTOS任务中能正常输出

## 额外功能

### SystemView集成 (可选)
如果需要SystemView功能，可以配置通道1：
```cpp
SEGGER_RTT_ConfigUpBuffer(1, "SysView", NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
```

### 双向通信 (可选)
RTT支持从主机向目标发送数据：
```cpp
char input_buffer[64];
int bytes_read = SEGGER_RTT_Read(0, input_buffer, sizeof(input_buffer));
if (bytes_read > 0) {
    // 处理接收到的数据
}
```

移植完成！现在您可以使用RTT进行高效的实时调试了。
