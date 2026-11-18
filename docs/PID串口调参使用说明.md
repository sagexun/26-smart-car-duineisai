# 智能车 PID 串口在线调参 使用说明（TC264 + 逐飞无线串口 + VOFA）

本说明教你：如何用上位机（电脑）通过无线串口，**在线实时修改**车上 PID 参数，不用每次改参数都重新编译下载。



***

## 一、整体原理（先搞懂数据流）



```
电脑上位机(VOFA/串口助手)

&#x20;       │  发送文本，例如 "P1=3.5!"

&#x20;       ▼

无线串口模块（蓝牙/2.4G）

&#x20;       │

&#x20;       ▼

TC264 芯片 UART\_2（P10\_5 发 / P10\_6 收）

&#x20;       │  硬件自动产生接收中断

&#x20;       ▼

isr.c 里的 uart2\_rx\_isr()

&#x20;       │  逐飞框架已自动调用 wireless\_module\_uart\_handler()

&#x20;       ▼

zf\_device\_wireless\_uart.c 把收到的字节存进内部 64 字节 FIFO

&#x20;       │

&#x20;       ▼

你在主循环里调用 pid\_tuning\_handler()   ←★ 我们这次新写的核心函数

&#x20;       │  从 FIFO 读出数据 → 组帧 → 解析 → 更新全局 NUM

&#x20;       ▼

你的 PID 算法直接用 NUM.angle\_p、NUM.speed\_p …… 就生效了
```

**重点：**



* 接收中断已经由逐飞库帮你写好了，`isr.c`**&#x20;不用改一行代码**。

* 无线串口初始化 `wireless_uart_init()` 已经在 `code/init.c` 里调用了，**也不用再初始化**。

* 你只需要做两件事：① 主循环里调用 `pid_tuning_handler()`；② 自己的 PID 算法里用 `NUM` 这个结构体。



***

## 二、数据包格式（上位机要发什么）

每一条调参指令都是一句**纯文本**，格式固定为：



```
环类型 + 环号 = 数值 !
```



| 要改的参数  | 上位机发送的文本    | 说明                      |
| ------ | ----------- | ----------------------- |
| 角度环 P  | `P1=3.5!`   | 把 NUM.angle\_p 改成 3.5   |
| 角度环 I  | `I1=0.02!`  | 把 NUM.angle\_i 改成 0.02  |
| 角度环 D  | `D1=12!`    | 把 NUM.angle\_d 改成 12    |
| 角速度环 P | `P2=2.0!`   | 把 NUM.anglev\_p 改成 2.0  |
| 角速度环 I | `I2=0!`     | 把 NUM.anglev\_i 改成 0    |
| 角速度环 D | `D2=0.5!`   | 把 NUM.anglev\_d 改成 0.5  |
| 速度环 P  | `P3=0.2!`   | 把 NUM.speed\_p 改成 0.2   |
| 速度环 I  | `I3=0.001!` | 把 NUM.speed\_i 改成 0.001 |
| 速度环 D  | `D3=0!`     | 把 NUM.speed\_d 改成 0     |

**规则：**



* 字母必须大写（`P` / `I` / `D`），环号只能是 `1`、`2`、`3`。

* 数值可以是小数，可以带负号，例如 `P1=-1.5!`。

* **结尾必须有一个英文感叹号&#x20;**`!`，它是 “一帧结束” 的标志，少了它芯片不会执行。

* 大小写字母之外的换行、回车都会被自动忽略，所以你发完直接回车也没关系。



***

## 三、代码改了哪些文件



| 文件                 | 改了什么                                                            | 你要不要动    |
| ------------------ | --------------------------------------------------------------- | -------- |
| `code/uart.h`      | 新增 `PIDNUM` 结构体、全局 `NUM`、函数声明                                   | 已写好，不用动  |
| `code/uart.c`      | 新增 `pid_tuning_handler()`、`pid_tuning_send_current()`，保留原来的测试函数 | 已写好，不用动  |
| `user/isr.c`       | 不用改（中断已挂好）                                                      | **不用动**  |
| `code/init.c`      | *不用改（*`wireless_uart_init()`*&#x20;已有）*                         | **不用动**  |
| `user/cpu0_main.c` | 需要在主循环里加一行 `pid_tuning_handler();`                              | **照下面改** |
| 你的 PID 算法文件        | 把参数引用改成 `NUM.xxx`                                               | 需要改      |



***

## 四、在 cpu0\_main.c 里接入（必做）

打开 `user/cpu0_main.c`，在 `while(TRUE)` 主循环里加上 `pid_tuning_handler();`。

把原来的主循环：



```
&#x20;   while (TRUE)

&#x20;   {

&#x20;       //这里写你要循环执行的程序

//        MotorL\_SetSpeed(1000);

//        MotorR\_SetSpeed(1000);

//        tft180\_show\_int(100, 110,motor\_L.encoder\_speed,  2);

//        tft180\_show\_int(100, 126,motor\_R.encoder\_speed,  2);

//        printf("%d,%d,%d,%d\n",motor\_L.encoder\_speed,motor\_R.encoder\_speed,motor\_L.target\_speed,motor\_R.target\_speed);

//        uart\_test\_transmit("Hello World\n");

&#x20;       wireless\_uart\_send\_buffer(buf,sizeof(buf));

&#x20;       system\_delay\_ms(500);

&#x20;   }
```

改成（关键是加了 `pid_tuning_handler();`，并把延时缩短，避免接收 FIFO 溢出）：



```
&#x20;   while (TRUE)

&#x20;   {

&#x20;       // ★ 新增：处理上位机发来的 PID 调参指令，必须频繁调用

&#x20;       pid\_tuning\_handler();

&#x20;       // --- 下面是你原来的代码，按需保留/注释 ---

&#x20;       // wireless\_uart\_send\_buffer(buf,sizeof(buf));

&#x20;       // system\_delay\_ms(500);   // 注意：延时不要太长！

&#x20;       system\_delay\_ms(1);         // 建议改成很短的延时，比如 1ms

&#x20;   }
```

> ⚠️ 
>
> **为什么延时要短？**
>
>  无线串口接收 FIFO 只有 64 字节。如果主循环里 
>
> `system_delay_ms(500)`
>
> ，这 500ms 里收到的数据会塞满 FIFO 导致丢包。
> 所以
>
> **调参期间主循环空延时一定要短**
>
> （1~5ms），或者干脆把 
>
> `pid_tuning_handler();`
>
>  放进一个 5ms 的定时中断（PIT）里调用。



***

## 五、在你自己的 PID 算法里用上 NUM

`NUM` 是一个全局变量，在 `uart.c` 里定义，已经在 `uart.h` 里 `extern` 声明了。

只要你的 PID 算法文件包含了 `uart.h`（或者通过 `zf_common_headfile.h` 间接包含），就能直接用：



```
// 例如你的位置式 PID 计算：

float p\_out = NUM.angle\_p \* err;            // 比例项

float i\_out = NUM.angle\_i \* err\_integral;   // 积分项

float d\_out = NUM.angle\_d \* (err - last\_err); // 微分项
```

如果你之前 PID 参数是自己定义的局部变量（比如 `float Kp = 3.0f;`），把它们全部替换成 `NUM.angle_p`、`NUM.speed_p` 等即可。



***

## 六、上位机（电脑端）怎么操作

### 方式 A：用任意串口助手（最简单，新手推荐）



1. 车上的无线串口模块插到主板，确认接线：

* 模块 VCC → 3.3V，GND → GND

* 模块 TX → 芯片 P10\_6（UART2\_RX）

* 模块 RX → 芯片 P10\_5（UART2\_TX）

* 模块 RTS → 芯片 P10\_2

1. 另一个无线串口模块插电脑 USB。

2. 打开串口助手（VOFA++ / 逐飞上位机 / 串口调试助手都行）：

* 波特率 **115200**，8 数据位，1 停止位，无校验。

1. 在发送框里输入指令，比如：



```
P1=3.5!
```

点发送，车上的 `NUM.angle_p` 就立刻变成 3.5。



1. 想确认参数有没有生效，可以在代码里调用 `pid_tuning_send_current();`，

   它会把当前 9 个参数通过无线串口发回来，在串口助手接收区就能看到。

### 方式 B：用 VOFA++ 的指令按钮 / 滑块（更顺手）

在 VOFA++ 里新建自定义按钮或滑块，把每个按钮 “发送字符串” 设成：



* 角度环 P 滑块 → 发送 `P1={val}!`（具体怎么把数值填进去看 VOFA++ 帮助，本质就是发上面那种文本）

* 其余同理（I1!、D1!、P2! ……）

拖动滑块时，VOFA 会自动把当前数值拼成 `P1=xxx!` 发出去，车上实时响应。



***

## 七、接线与参数速查（来自 zf\_device\_wireless\_uart.h）



| 项目       | 值                 |
| -------- | ----------------- |
| 使用的串口号   | UART\_2           |
| 波特率      | 115200            |
| 芯片发送脚 TX | P10\_5（接模块 RX）    |
| 芯片接收脚 RX | P10\_6（接模块 TX）    |
| RTS 流控脚  | P10\_2            |
| 自动波特率    | 已开启（模块 V2.0 自动匹配） |

如果你的无线串口模块引脚和上面不一致，去改 `libraries/zf_device/zf_device_wireless_uart.h` 最上面的 `#define` 即可。



***

## 八、调试步骤（建议按顺序来）



1. 先只下载本工程，打开串口助手，确认能收到 `SEEKFREE wireless test.` 或你发的回显 → 说明无线链路通了。

2. 在主循环加上 `pid_tuning_handler();`，下载。

3. 串口助手发送 `P1=9.9!`，再调用一次 `pid_tuning_send_current();`（或周期性调用），

   看接收区里 `P1` 是不是变成了 `9.900`。变成了 → 收发链路完全 OK。

4. 把 `NUM.xxx` 接到你的 PID 算法里，开始调参：

* 一般先调角度环（P1→D1），再调角速度环（P2→D2），最后调速度环（P3→I3）。

1. 调好后，把最终数值抄回 `uart.c` 里 `PIDNUM NUM = {...}` 的默认值，这样不接上位机也能跑。



***

## 九、常见问题

**Q：发了指令没反应？**



* 检查结尾的 `!` 是不是英文感叹号，不要用中文 `！`。

* 检查主循环里 `pid_tuning_handler();` 有没有真的被频繁调用（延时别太长）。

* 检查波特率是不是 115200，串口号选对没有。

**Q：收到的参数不对 / 跳来跳去？**



* 大概率是接线问题：模块 TX/RX 接反了，或者 GND 没共地。

* 或者主循环间隔太久，FIFO 溢出丢字节，导致帧错位。把延时缩短再试。

**Q：能不能用 printf 回显？**



* 可以，但要注意：`printf` 默认走的是调试串口（UART0），不是无线串口。要通过无线串口回显请用 `wireless_uart_send_string()`，也就是本工程里的 `pid_tuning_send_current()`。

**Q：要不要改 isr.c？**



* 不要。UART2 接收中断在 `isr.c` 第 213 行已经调用了 `wireless_module_uart_handler()`，逐飞库内部会自动把字节送进 FIFO。