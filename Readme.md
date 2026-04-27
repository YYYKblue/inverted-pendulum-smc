# 倒立摆验证UAS-SMC  

## 使用vscode+keil作为开发环境  

使用**Keil uVision Assistant** 插件进行开发

[VSCode 与 Keil 协同嵌入式开发](https://alayedong.cn/posts/2024/vscode-and-keil-collaborative-embedded-development/)

## 开发中遇到的问题  

### st-link直插无法给主板供电  

STLink的手册里怎么描述1和2脚的，人家叫TVCC，**Target VCC**，什么意思啊，就是说这个脚是接到目标芯片的IO轨电源上，用来**检测**IO顺从电压以确保信号的兼容性。因为debug工具不知道你的单板上IO供电电压到底是多少，所以需要检测一下，然后做出相应的调整，而不是通过这个接口给目标单板供电。

**因此需要额外单独的供电才能烧录调试**

### 为啥使用串口中断的时候要增加当前串口判断  
因为不同串口共用一个中断服务函数，也就是共用一套中断回调函数，必须根据所给的串口句柄来判断数据来源，否则解析会出错  

