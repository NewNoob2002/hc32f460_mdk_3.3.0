.syntax unified    @ 使用统一汇编语法
.arch armv7e-m    @ 指定架构（根据实际目标调整）

.section .text    @ 定义在.text段
.global __use_no_semihosting  @ 关键声明

@ 可选：添加空标签避免警告
__use_no_semihosting:
    bx lr       @ 空函数（仅占位，实际不会调用）