#ifndef __display_hpp__
#define __display_hpp__

#define BIN_THRESHOLD   128           // 二值化默认阈值（备用，大津法接管后不再用）

void display_data(uint8 bin_threshold = BIN_THRESHOLD);
void display_image(uint8 bin_threshold = BIN_THRESHOLD);   // 传入大津法阈值
void display_edge(void);                                    // 边线叠加显示
#endif
