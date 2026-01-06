/**
Edit by modify: Ngoc Hang
**/
#include <stdio.h>  // Cho snprintf
#include <string.h>
#include <stdlib.h>
#include "lcd_i2c.h"
#include "globals.h"
extern I2C_HandleTypeDef hi2c2;  // change your handler here accordingly

#define SLAVE_ADDRESS_LCD 0x27 << 1 // change this according to ur setup
#define debounce 5
char disp_buf[21];
char conf_buf[21];
uint8_t pos_Addr;
char data_u, data_l;
uint8_t data_t[4];
char time_buf[9];


void lcd_send_cmd (char cmd)
{
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_init (void)
{
	// Khởi tạo 4-bit mode cho HD44780 qua I2C
	HAL_Delay(2);  // Đợi LCD ổn định
	lcd_send_cmd (0x03);  // Sequence đặc biệt cho 4-bit init (gốc code có 0x33/0x32, điều chỉnh cho chuẩn)
	HAL_Delay(1);
	lcd_send_cmd (0x03);
	HAL_Delay(1);
	lcd_send_cmd (0x03);
	HAL_Delay(1);
	lcd_send_cmd (0x02);  // Set 4-bit mode
	HAL_Delay(1);
	lcd_send_cmd (0x28);  // Function set: 4-bit, 2 lines (nhưng cho 20x4 vẫn dùng, controller hỗ trợ)
	HAL_Delay(1);
	lcd_send_cmd (0x0C);  // Display on, cursor off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // Clear display
	HAL_Delay(2);  // Clear cần >1.5ms
	lcd_send_cmd (0x06);  // Entry mode set: increment, no shift
	HAL_Delay(1);
}

void lcd_send_string (const char *str)
{
	while (*str) lcd_send_data (*str++);
}

void lcd_print_uint32(int row, int col,uint32_t value) {
	char buf[11];  // Đủ cho 4294967295 (10 chữ số) + '\0'
	snprintf(buf, sizeof(buf), "%lu", value);  // %lu cho unsigned long (uint32_t)
	lcd_goto_XY(row, col);  // Đặt vị trí trước
  	lcd_send_string(buf);
  	HAL_Delay(2);

	// Clear phần còn lại của dòng nếu cần (tùy vị trí)
}
void lcd_display_rpm_input(void) {
  snprintf(disp_buf, sizeof(disp_buf), "RPM:%s ", buffer);  // %s cho buffer, thêm space để xóa ký tự cũ
  lcd_goto_XY(1, 0);  // Dòng 1, cột 0
  lcd_send_string(disp_buf);
  HAL_Delay(2);
}
// Hàm hiển thị thông báo xác nhận (sau enter hoặc thay đổi)
void lcd_display_confirmation(uint8_t row, uint8_t col,const char* msg, uint32_t value) {
  snprintf(conf_buf, sizeof(conf_buf), "%s:%lu", msg, value);
  lcd_goto_XY(row, col);
  lcd_send_string(conf_buf);
  HAL_Delay(2);
}
void lcd_display_float_light(uint8_t row, uint8_t col,const char* msg, float value)
{
    int int_part = (int)value;
    int frac_part = (int)((value - int_part) * 100); // 2 chữ số

    char buf[21];
    snprintf(buf, sizeof(buf), "%s:%d.%01d", msg, int_part, frac_part);

    lcd_goto_XY(row, col);
    lcd_send_string(buf);
    HAL_Delay(2);
    HAL_Delay(1500);
}

void lcd_clear_display (void)
{
	lcd_send_cmd(0x01);
	HAL_Delay(2);
}

void lcd_goto_XY (int row, int col)
{

	switch (row)
	    {
	        case 0:
	            pos_Addr = 0x80 + col;        // dòng 1: 0x00
	            break;
	        case 1:
	            pos_Addr = 0x80 + 0x40 + col; // dòng 2: 0x40
	            break;
	        case 2:
	            pos_Addr = 0x80 + 0x14 + col; // dòng 3: 0x14
	            break;
	        case 3:
	            pos_Addr = 0x80 + 0x54 + col; // dòng 4: 0x54
	            break;
	        default:
	            pos_Addr = 0x80;
	    }

	lcd_send_cmd(pos_Addr);
	HAL_Delay(2);
}

void get_motor_runtime(char* time_str, size_t buf_size) {
  if (!motor_running) {
    snprintf(time_str, buf_size, "00:00:00");
    return;
  }

  uint32_t now = HAL_GetTick();
  uint32_t delta_ms = now - motor_start_time;
  uint32_t total_seconds = delta_ms / 1000;

  uint32_t hours = (total_seconds / 3600) % 100;  // Giới hạn 99 giờ
  uint32_t minutes = (total_seconds / 60) % 60;
  uint32_t seconds = total_seconds % 60;

  snprintf(time_str, buf_size, "%02lu:%02lu:%02lu", hours, minutes, seconds);
}

void lcd_update_runtime(void) {

  if (!motor_running) {
    snprintf(time_buf, sizeof(time_buf), "00:00:00");
  } else {
    uint32_t delta_ms = HAL_GetTick() - motor_start_time;
    uint32_t total_sec = delta_ms / 1000;
    snprintf(time_buf, sizeof(time_buf), "%02lu:%02lu:%02lu",
             (total_sec / 3600) % 100, (total_sec / 60) % 60, total_sec % 60);
  }

  lcd_goto_XY(2, 0);
  for (int i = 0; i < 20; i++) lcd_send_data(' ');
  lcd_goto_XY(2, 0);
  lcd_send_string("Time: ");
  lcd_send_string(time_buf);
}
