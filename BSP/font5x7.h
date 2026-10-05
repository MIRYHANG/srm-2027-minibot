//
// Created by YZH on 2026/10/5.
//

#ifndef XIAOSAI_FONT5X7_H
#define XIAOSAI_FONT5X7_H

#include <stdint.h>

// 每个字形 5 列，每列 1 字节，最低位在最上面，只用低 7 位
#define FONT5X7_WIDTH 5U

/**
 * @brief 取一个字符的字形
 * @return 指向 FONT5X7_WIDTH 个字节的只读数组，永远不为 NULL
 * @note 只收录空格到 'Z'；小写字母按大写显示，其他字符显示为 '?'
 */
const uint8_t *Font5x7_Glyph(char c);

#endif //XIAOSAI_FONT5X7_H