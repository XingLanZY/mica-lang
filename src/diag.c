#include "diag.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TAB_STOP 8        /* tab 展开后的列距 */
#define MAX_SHOWN 80      /* 单行最大显示列数，超出截断 */
#define CARET_MARGIN 20   /* 截断时箭头前保留的最小列数 */
#define MAX_CHARS 4096    /* 单行最多解析字符数 */

typedef struct {
    const unsigned char *p;   /* 字节起点 */
    size_t n;                 /* 字节长度 */
    int col;                  /* 显示列（0 基） */
    int width;                /* 显示宽度（tab 为展开后宽度） */
} LineChar;

/* ---------- UTF-8 解码 ---------- */

static uint32_t utf8_decode(const unsigned char *s, size_t n, size_t *adv) {
    if (n == 0) {
        *adv = 0;
        return 0;
    }
    unsigned char b0 = s[0];
    if (b0 < 0x80) {
        *adv = 1;
        return b0;
    }
    size_t need;
    uint32_t cp;
    if ((b0 & 0xE0) == 0xC0) {
        need = 1;
        cp = b0 & 0x1F;
    } else if ((b0 & 0xF0) == 0xE0) {
        need = 2;
        cp = b0 & 0x0F;
    } else if ((b0 & 0xF8) == 0xF0) {
        need = 3;
        cp = b0 & 0x07;
    } else {
        *adv = 1;
        return b0;
    }
    if (n < need + 1) {
        *adv = 1;
        return b0;
    }
    for (size_t i = 1; i <= need; i++) {
        unsigned char bc = s[i];
        if ((bc & 0xC0) != 0x80) {
            *adv = 1;
            return b0;
        }
        cp = (cp << 6) | (bc & 0x3F);
    }
    /* 拒绝过短编码 / 代理区 / 越界 */
    if ((need == 1 && cp < 0x80) || (need == 2 && cp < 0x800) ||
        (need == 3 && cp < 0x10000) || (cp >= 0xD800 && cp <= 0xDFFF) ||
        cp > 0x10FFFF) {
        *adv = 1;
        return b0;
    }
    *adv = need + 1;
    return cp;
}

/* ---------- 显示宽度（按东亚宽字符启发式；歧义字符按 1） ---------- */

static int char_width(uint32_t cp) {
    if (cp < 0x20 || cp == 0x7f) return 0;
    if (cp < 0x7f) return 1;
    if ((cp >= 0x1100 && cp <= 0x115F) ||   /* Hangul Jamo */
        (cp >= 0x2E80 && cp <= 0x303E) ||   /* CJK Radicals / Kangxi / Symbols */
        (cp >= 0x3041 && cp <= 0x33FF) ||   /* Hiragana / Katakana / CJK Compat */
        (cp >= 0x3400 && cp <= 0x4DBF) ||   /* CJK Ext A */
        (cp >= 0x4E00 && cp <= 0x9FFF) ||   /* CJK Unified */
        (cp >= 0xA000 && cp <= 0xA4CF) ||   /* Yi */
        (cp >= 0xAC00 && cp <= 0xD7A3) ||   /* Hangul Syllables */
        (cp >= 0xF900 && cp <= 0xFAFF) ||   /* CJK Compat Ideographs */
        (cp >= 0xFE10 && cp <= 0xFE19) ||
        (cp >= 0xFE30 && cp <= 0xFE6F) ||
        (cp >= 0xFF00 && cp <= 0xFF60) ||   /* Fullwidth Forms */
        (cp >= 0xFFE0 && cp <= 0xFFE6) ||
        (cp >= 0x1F000 && cp <= 0x1FAFF) || /* 常见 Emoji 按宽 */
        (cp >= 0x20000 && cp <= 0x3FFFD))   /* CJK Ext B+ */
        return 2;
    return 1;
}

/* ---------- 行定位 ---------- */

static const unsigned char *find_line(const unsigned char *src, size_t len,
                                      int line, const unsigned char **out_end) {
    const unsigned char *p = src;
    const unsigned char *end = src + len;
    int cur = 1;
    while (cur < line && p < end) {
        if (*p == '\n') {
            cur++;
            p++;
        } else if (*p == '\r') {
            cur++;
            p++;
            if (p < end && *p == '\n') p++;
        } else {
            p++;
        }
    }
    const unsigned char *e = p;
    while (e < end && *e != '\n' && *e != '\r') e++;
    *out_end = e;
    return p;
}

/* ---------- 行解析（UTF-8 + tab 展开） ---------- */

static int parse_line(const unsigned char *start, const unsigned char *end, LineChar *chars) {
    int count = 0;
    int col = 0;
    const unsigned char *p = start;
    while (p < end && count < MAX_CHARS) {
        LineChar *lc = &chars[count];
        lc->p = p;
        lc->col = col;
        if (*p == '\t') {
            lc->n = 1;
            lc->width = TAB_STOP - (col % TAB_STOP);
            col += lc->width;
            p++;
        } else if (*p < 0x20 || *p == 0x7f) {
            lc->n = 1;
            lc->width = 1;   /* 控制字符渲染为 '.' */
            col += 1;
            p++;
        } else {
            size_t adv = 0;
            uint32_t cp = utf8_decode(p, (size_t)(end - p), &adv);
            lc->n = adv;
            lc->width = char_width(cp);
            col += lc->width;
            p += adv;
        }
        count++;
    }
    return count;
}

static void print_lchar(const LineChar *lc) {
    if (*lc->p == '\t') {
        for (int i = 0; i < lc->width; i++) putchar(' ');
    } else if (*lc->p < 0x20 || *lc->p == 0x7f) {
        putchar('.');
    } else {
        fwrite(lc->p, 1, lc->n, stdout);   /* 原样输出 UTF-8 字节 */
    }
}

/* ---------- 主入口 ---------- */

void diag_print_error(const char *source, size_t len,
                      const char *filename, const LexError *err) {
    if (!err || !err->has_error) return;
    const char *file = (filename && filename[0]) ? filename : "<input>";
    int line = err->line > 0 ? err->line : 1;
    int col = err->col > 0 ? err->col : 1;
    int span = err->span > 0 ? err->span : 1;

    printf("%s:%d:%d: error: %s\n", file, line, col, err->message);

    const unsigned char *line_end = NULL;
    const unsigned char *line_start = find_line((const unsigned char *)source, len, line, &line_end);
    LineChar chars[MAX_CHARS];
    int n = parse_line(line_start, line_end, chars);

    size_t line_len = (size_t)(line_end - line_start);
    size_t err_byte = (size_t)(col - 1);
    if (err_byte > line_len) err_byte = line_len;

    int total_width = n ? chars[n - 1].col + chars[n - 1].width : 0;

    /* 定位出错字符 */
    int caret_idx = -1;
    int caret_col = total_width;
    for (int i = 0; i < n; i++) {
        size_t start = (size_t)(chars[i].p - line_start);
        if (err_byte < start + chars[i].n) {
            caret_idx = i;
            caret_col = chars[i].col;
            break;
        }
    }

    /* 长行截断：以出错处为中心开一个窗口 */
    int start_idx = 0, end_idx = n;
    bool trunc_front = false, trunc_back = false;
    int shown_start_col = 0;
    if (total_width > MAX_SHOWN) {
        int win_start_col = caret_col - CARET_MARGIN;
        if (win_start_col < 0) win_start_col = 0;
        int win_end_col = win_start_col + MAX_SHOWN;
        while (start_idx < n && chars[start_idx].col < win_start_col) start_idx++;
        trunc_front = start_idx > 0;
        shown_start_col = (start_idx < n) ? chars[start_idx].col : win_start_col;
        end_idx = start_idx;
        while (end_idx < n && chars[end_idx].col < win_end_col) end_idx++;
        trunc_back = end_idx < n;
    }

    /* 行号 gutter 宽度 */
    int gutter_w = 1;
    for (int t = line; t > 0; t /= 10) gutter_w++;

    /* 源码行 */
    printf("%*d | ", gutter_w, line);
    if (trunc_front) fputs("...", stdout);
    for (int i = start_idx; i < end_idx; i++) print_lchar(&chars[i]);
    if (trunc_back) fputs("...", stdout);
    putchar('\n');

    /* 箭头行 */
    int caret_off = (trunc_front ? 3 : 0) + (caret_col - shown_start_col);
    if (caret_off < 0) caret_off = 0;
    int shown_width = (trunc_front ? 3 : 0) +
                      (end_idx > start_idx ? (chars[end_idx - 1].col + chars[end_idx - 1].width - shown_start_col) : 0) +
                      (trunc_back ? 3 : 0);

    printf("%*s | ", gutter_w, "");
    for (int i = 0; i < caret_off; i++) putchar(' ');
    putchar('^');

    /* 下划线覆盖出错 token（按显示宽度） */
    int range = 1;
    if (caret_idx >= 0) {
        size_t consumed = 0;
        range = 0;
        for (int i = caret_idx; i < n && consumed < (size_t)span; i++) {
            range += chars[i].width;
            consumed += chars[i].n;
        }
        if (range < 1) range = 1;
    }
    int avail = shown_width - caret_off;
    if (range > avail) range = avail;
    if (range < 1) range = 1;
    for (int i = 1; i < range; i++) putchar('~');
    putchar('\n');
}
