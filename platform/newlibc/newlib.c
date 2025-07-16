/**
 * stubs for compatibility with newlib.
 * as per https://sourceware.org/newlib/libc.html#Stubs
 */
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include "CommonMacro.h"

// 定义必要的常量（如果未定义）
#ifndef STDOUT_FILENO
#define STDOUT_FILENO 1
#endif

#ifndef STDERR_FILENO
#define STDERR_FILENO 2
#endif

#ifndef S_IFCHR
#define S_IFCHR 0x2000
#endif

// 简化的stat结构体定义
struct stat {
    int st_mode;
};

int usart1_write(uint8_t *ch, int len);

// AC6编译器printf重定向实现
#ifdef __ARMCC_VERSION

/**
 * @brief 重定向_write函数到串口输出
 * @param fd 文件描述符
 * @param data 数据指针
 * @param size 数据大小
 * @retval 实际写入的字节数，失败时返回-1
 */
int _write(int fd, char *data, int size)
{
    int i;
    int written = 0;
    
    // 检查参数有效性
    if (data == NULL || size <= 0) {
        return -1;
    }
    
    // 只处理标准输出和标准错误
    if (fd != STDOUT_FILENO && fd != STDERR_FILENO) {
        return -1;
    }
    
    // 使用usart1_write函数输出数据
    for (i = 0; i < size; i++) {
        // 添加安全检查，避免在串口未初始化时调用
        int result = usart1_write((uint8_t*)&data[i], 1);
        if (result != 1) {
            // 如果写入失败，返回已写入的字节数
            // 在串口未初始化时，返回0而不是-1，避免错误
            return written;
        }
        written++;
    }
    
    return written;
}

/**
 * @brief 重定向fputc函数到串口输出
 * @param ch 要输出的字符
 * @param stream 文件流
 * @retval 成功时返回字符，失败时返回EOF
 */
int fputc(int ch, FILE *stream)
{
    (void)stream;  // 忽略stream参数
    
    if (usart1_write((uint8_t*)&ch, 1) == 1) {
        return ch;
    } else {
        return EOF;
    }
}

/**
 * @brief 重定向_ttywrch函数（终端写入字符）
 * @param ch 要写入的字符
 * @retval 无返回值
 */
void _ttywrch(int ch)
{
    // 在系统初始化阶段，串口可能还未初始化
    // 暂时忽略输出，避免硬件错误
    (void)ch;
    // 注意：在串口初始化完成后，printf会通过_write函数输出
    // 这里只是为了避免系统初始化时的硬件错误
}

/**
 * @brief 重定向_sys_exit函数（系统退出）
 * @param status 退出状态
 * @retval 无返回值
 */
void _sys_exit(int status)
{
    (void)status;
    // 在嵌入式系统中，不执行真正的退出
    // 而是返回到调用者，避免死循环
    return;
}

/**
 * @brief 重定向_sys_open函数（系统打开文件）
 * @param path 文件路径
 * @param mode 打开模式
 * @retval -1（失败）
 */
__attribute__((weak)) int _sys_open(const char *path, int mode)
{
    (void)path;
    (void)mode;
    return -1;  // 不支持文件操作
}

/**
 * @brief 重定向_sys_close函数（系统关闭文件）
 * @param fd 文件描述符
 * @retval 0（成功）
 */
__attribute__((weak)) int _sys_close(int fd)
{
    (void)fd;
    return 0;
}

/**
 * @brief 重定向_sys_write函数（系统写入文件）
 * @param fd 文件描述符
 * @param data 数据指针
 * @param size 数据大小
 * @retval 实际写入的字节数
 */
__attribute__((weak)) int _sys_write(int fd, const char *data, int size)
{
    (void)fd;
    (void)data;
    (void)size;
    return size;  // 假设成功写入
}

/**
 * @brief 重定向_sys_read函数（系统读取文件）
 * @param fd 文件描述符
 * @param data 数据指针
 * @param size 数据大小
 * @retval 0（无数据可读）
 */
__attribute__((weak)) int _sys_read(int fd, char *data, int size)
{
    (void)fd;
    (void)data;
    (void)size;
    return 0;  // 无数据可读
}

/**
 * @brief 重定向_sys_flen函数（系统文件长度）
 * @param fd 文件描述符
 * @retval 0（文件长度为0）
 */
__attribute__((weak)) long _sys_flen(int fd)
{
    (void)fd;
    return 0;  // 文件长度为0
}

/**
 * @brief 重定向_sys_istty函数（系统是否为终端）
 * @param fd 文件描述符
 * @retval 1（是终端）
 */
__attribute__((weak)) int _sys_istty(int fd)
{
    return (fd == STDOUT_FILENO || fd == STDERR_FILENO) ? 1 : 0;
}

/**
 * @brief 重定向_sys_seek函数（系统文件定位）
 * @param fd 文件描述符
 * @param pos 位置
 * @retval 0（成功）
 */
__attribute__((weak)) long _sys_seek(int fd, long pos)
{
    (void)fd;
    (void)pos;
    return 0;  // 成功
}

/**
 * @brief 重定向_sys_ensure函数（系统确保写入）
 * @param fd 文件描述符
 * @retval 0（成功）
 */
__attribute__((weak)) int _sys_ensure(int fd)
{
    (void)fd;
    return 0;  // 成功
}

#endif

 