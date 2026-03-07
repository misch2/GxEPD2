// Panel: ZJY800480-075BABMFGN-R: FIXME
// Controller: UC8179

#include "GxEPD2_750c_ZJY.h"

GxEPD2_750c_ZJY::GxEPD2_750c_ZJY(int16_t cs, int16_t dc, int16_t rst, int16_t busy) :
  GxEPD2_EPD(cs, dc, rst, busy, LOW, 30000000, WIDTH, HEIGHT, panel, hasColor, hasPartialUpdate, hasFastPartialUpdate)
{
}

void GxEPD2_750c_ZJY::clearScreen(uint8_t value)
{
  Serial.println("[michal FIXME] clearScreen() start");
  writeScreenBuffer(value);
  refresh(false);
  writeScreenBuffer(value);
  Serial.println("[michal FIXME] clearScreen() end");
}

void GxEPD2_750c_ZJY::writeScreenBuffer(uint8_t value)
{
  Serial.printf("[michal FIXME] writeScreenBuffer(0x%02X) start, using_partial_mode=%d, initial_refresh=%d\n", value, _using_partial_mode, _initial_refresh);
  if (!_using_partial_mode) _Init_Part();
  _writeScreenBuffer(0x13, value); // set current
  if (_initial_refresh) _writeScreenBuffer(0x10, 0x00); // set previous
  _initial_write = false; // initial full screen buffer clean done
  Serial.println("[michal FIXME] writeScreenBuffer() end");
}

void GxEPD2_750c_ZJY::_writeScreenBuffer(uint8_t command, uint8_t value)
{
  Serial.printf("[michal FIXME] _writeScreenBuffer(0x%02X, 0x%02X) start\n", command, value);
  _writeCommand(command);
  _startTransfer();
  for (uint32_t i = 0; i < uint32_t(WIDTH) * uint32_t(HEIGHT) / 8; i++)
  {
    _transfer(value);
  }
  _endTransfer();
  Serial.println("[michal FIXME] _writeScreenBuffer() end");
}

void GxEPD2_750c_ZJY::writeImage(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  _writeImage(0x13, bitmap, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_750c_ZJY::writeImageForFullRefresh(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  _writeImage(0x13, bitmap, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_750c_ZJY::_writeImage(uint8_t command, const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (_initial_write) writeScreenBuffer(); // initial full screen buffer clean
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32
  uint16_t wb = (w + 7) / 8; // width bytes, bitmaps are padded
  x -= x % 8; // byte boundary
  w = wb * 8; // byte boundary
  int16_t x1 = x < 0 ? 0 : x; // limit
  int16_t y1 = y < 0 ? 0 : y; // limit
  int16_t w1 = x + w < int16_t(WIDTH) ? w : int16_t(WIDTH) - x; // limit
  int16_t h1 = y + h < int16_t(HEIGHT) ? h : int16_t(HEIGHT) - y; // limit
  int16_t dx = x1 - x;
  int16_t dy = y1 - y;
  w1 -= dx;
  h1 -= dy;
  if ((w1 <= 0) || (h1 <= 0)) return;
  if (!_using_partial_mode) _Init_Part();
  _writeCommand(0x91); // partial in
  _setPartialRamArea(x1, y1, w1, h1);
  _writeCommand(command);
  _startTransfer();
  for (int16_t i = 0; i < h1; i++)
  {
    for (int16_t j = 0; j < w1 / 8; j++)
    {
      uint8_t data;
      // use wb, h of bitmap for index!
      uint16_t idx = mirror_y ? j + dx / 8 + uint16_t((h - 1 - (i + dy))) * wb : j + dx / 8 + uint16_t(i + dy) * wb;
      if (pgm)
      {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
        data = pgm_read_byte(&bitmap[idx]);
#else
        data = bitmap[idx];
#endif
      }
      else
      {
        data = bitmap[idx];
      }
      if (invert) data = ~data;
      _transfer(data);
    }
  }
  _endTransfer();
  _writeCommand(0x92); // partial out
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32
}

void GxEPD2_750c_ZJY::writeImagePart(const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
    int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  _writeImagePart(0x13, bitmap, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
}

void GxEPD2_750c_ZJY::_writeImagePart(uint8_t command, const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
    int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (_initial_write) writeScreenBuffer(); // initial full screen buffer clean
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32
  if ((w_bitmap < 0) || (h_bitmap < 0) || (w < 0) || (h < 0)) return;
  if ((x_part < 0) || (x_part >= w_bitmap)) return;
  if ((y_part < 0) || (y_part >= h_bitmap)) return;
  uint16_t wb_bitmap = (w_bitmap + 7) / 8; // width bytes, bitmaps are padded
  x_part -= x_part % 8; // byte boundary
  w = w_bitmap - x_part < w ? w_bitmap - x_part : w; // limit
  h = h_bitmap - y_part < h ? h_bitmap - y_part : h; // limit
  x -= x % 8; // byte boundary
  w = 8 * ((w + 7) / 8); // byte boundary, bitmaps are padded
  int16_t x1 = x < 0 ? 0 : x; // limit
  int16_t y1 = y < 0 ? 0 : y; // limit
  int16_t w1 = x + w < int16_t(WIDTH) ? w : int16_t(WIDTH) - x; // limit
  int16_t h1 = y + h < int16_t(HEIGHT) ? h : int16_t(HEIGHT) - y; // limit
  int16_t dx = x1 - x;
  int16_t dy = y1 - y;
  w1 -= dx;
  h1 -= dy;
  if ((w1 <= 0) || (h1 <= 0)) return;
  if (!_using_partial_mode) _Init_Part();
  _writeCommand(0x91); // partial in
  _setPartialRamArea(x1, y1, w1, h1);
  _writeCommand(command);
  _startTransfer();
  for (int16_t i = 0; i < h1; i++)
  {
    for (int16_t j = 0; j < w1 / 8; j++)
    {
      uint8_t data;
      // use wb_bitmap, h_bitmap of bitmap for index!
      uint16_t idx = mirror_y ? x_part / 8 + j + dx / 8 + uint16_t((h_bitmap - 1 - (y_part + i + dy))) * wb_bitmap : x_part / 8 + j + dx / 8 + uint16_t(y_part + i + dy) * wb_bitmap;
      if (pgm)
      {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
        data = pgm_read_byte(&bitmap[idx]);
#else
        data = bitmap[idx];
#endif
      }
      else
      {
        data = bitmap[idx];
      }
      if (invert) data = ~data;
      _transfer(data);
    }
  }
  _endTransfer();
  _writeCommand(0x92); // partial out
  delay(1); // yield() to avoid WDT on ESP8266 and ESP32
}

void GxEPD2_750c_ZJY::writeImage(const uint8_t* black, const uint8_t* color, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (black)
  {
    writeImage(black, x, y, w, h, invert, mirror_y, pgm);
  }
}

void GxEPD2_750c_ZJY::writeImagePart(const uint8_t* black, const uint8_t* color, int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
    int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (black)
  {
    writeImagePart(black, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
  }
}

void GxEPD2_750c_ZJY::writeNative(const uint8_t* data1, const uint8_t* data2, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  if (data1)
  {
    writeImage(data1, x, y, w, h, invert, mirror_y, pgm);
  }
}

void GxEPD2_750c_ZJY::drawImage(const uint8_t bitmap[], int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeImage(bitmap, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
}

void GxEPD2_750c_ZJY::drawImagePart(const uint8_t bitmap[], int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
    int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeImagePart(bitmap, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
}

void GxEPD2_750c_ZJY::drawImage(const uint8_t* black, const uint8_t* color, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeImage(black, color, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
}

void GxEPD2_750c_ZJY::drawImagePart(const uint8_t* black, const uint8_t* color, int16_t x_part, int16_t y_part, int16_t w_bitmap, int16_t h_bitmap,
    int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeImagePart(black, color, x_part, y_part, w_bitmap, h_bitmap, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
}

void GxEPD2_750c_ZJY::drawNative(const uint8_t* data1, const uint8_t* data2, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y, bool pgm)
{
  writeNative(data1, data2, x, y, w, h, invert, mirror_y, pgm);
  refresh(x, y, w, h);
}

void GxEPD2_750c_ZJY::refresh(bool partial_update_mode)
{
  Serial.printf("[michal FIXME] refresh(partial_update=%d) start\n", partial_update_mode);
  if (partial_update_mode) refresh(0, 0, WIDTH, HEIGHT);
  else
  {
    if (_using_partial_mode) _Init_Full();
    _Update_Full();
    _initial_refresh = false; // initial full update done
  }
  Serial.println("[michal FIXME] refresh() end");
}

void GxEPD2_750c_ZJY::refresh(int16_t x, int16_t y, int16_t w, int16_t h)
{
  if (_initial_refresh) return refresh(false); // initial update needs be full update
  // intersection with screen
  int16_t w1 = x < 0 ? w + x : w; // reduce
  int16_t h1 = y < 0 ? h + y : h; // reduce
  int16_t x1 = x < 0 ? 0 : x; // limit
  int16_t y1 = y < 0 ? 0 : y; // limit
  w1 = x1 + w1 < int16_t(WIDTH) ? w1 : int16_t(WIDTH) - x1; // limit
  h1 = y1 + h1 < int16_t(HEIGHT) ? h1 : int16_t(HEIGHT) - y1; // limit
  if ((w1 <= 0) || (h1 <= 0)) return;
  // make x1, w1 multiple of 8
  w1 += x1 % 8;
  if (w1 % 8 > 0) w1 += 8 - w1 % 8;
  x1 -= x1 % 8;
  if (!_using_partial_mode) _Init_Part();
  if (usePartialUpdateWindow) _writeCommand(0x91); // partial in
  _setPartialRamArea(x1, y1, w1, h1);
  _Update_Part();
  if (usePartialUpdateWindow) _writeCommand(0x92); // partial out
}

void GxEPD2_750c_ZJY::powerOff(void)
{
  _PowerOff();
}

void GxEPD2_750c_ZJY::hibernate()
{
  Serial.println("[michal FIXME] hibernate() start");
  _PowerOff();
  if (_rst >= 0)
  {
    _writeCommand(0x07); // deep sleep
    _writeData(0xA5);    // check code
    _hibernating = true;
  }
  Serial.println("[michal FIXME] hibernate() end");
}

void GxEPD2_750c_ZJY::_setPartialRamArea(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  uint16_t xe = (x + w - 1) | 0x0007; // byte boundary inclusive (last byte)
  uint16_t ye = y + h - 1;
  x &= 0xFFF8; // byte boundary
  Serial.printf("[michal FIXME] setting partial RAM area: x=%d, y=%d, w=%d, h=%d\n", x, y, w, h);
  _writeCommand(0x90); // partial window
  _writeData(x / 256);
  _writeData(x % 256);
  _writeData(xe / 256);
  _writeData(xe % 256);
  _writeData(y / 256);
  _writeData(y % 256);
  _writeData(ye / 256);
  _writeData(ye % 256);
  _writeData(0x01);
}

void GxEPD2_750c_ZJY::_PowerOn()
{
  Serial.println("[michal FIXME] _PowerOn() start");
  if (!_power_is_on)
  {
    _writeCommand(0x04);
    _waitWhileBusy("_PowerOn", power_on_time);
  } else {
    Serial.println("[michal FIXME] _PowerOn() skipped, already powered on");
  }
  _power_is_on = true;
  Serial.println("[michal FIXME] _PowerOn() end");
}

void GxEPD2_750c_ZJY::_PowerOff()
{
  Serial.println("[michal FIXME] _PowerOff() start");
  if (_power_is_on)
  {
    _writeCommand(0x02); // power off
    _waitWhileBusy("_PowerOff", power_off_time);
  } else {
    Serial.println("[michal FIXME] _PowerOff() skipped, already powered off");
  }
  _power_is_on = false;
  _using_partial_mode = false;
  Serial.println("[michal FIXME] _PowerOff() end");
}

void GxEPD2_750c_ZJY::_InitDisplay()
{
  Serial.println("[michal FIXME] _InitDisplay() start");
  Serial.println("[michal FIXME] V=4");
  if (_hibernating) _reset();
  _writeCommand(0x00); // PANEL SETTING
  _writeData(0x0f);    // KW: 3f, KWR: 2F, BWROTP: 0f, BWOTP: 1f
  // _writeCommand(0x50); // VCOM AND DATA INTERVAL SETTING
  // _writeData(0x29);
  // _writeData(0x07);
  Serial.println("[michal FIXME] _InitDisplay() end");
}

void GxEPD2_750c_ZJY::_Init_Full()
{
  Serial.println("[michal FIXME] _Init_Full() start");
  _InitDisplay();
  _writeCommand(0x00); // panel setting
  _writeData(0x0f);    // full update LUT from OTP
  _PowerOn();
  _using_partial_mode = false;
  Serial.println("[michal FIXME] _Init_Full() end");
}

void GxEPD2_750c_ZJY::_Init_Part()
{
  Serial.println("[michal FIXME] _Init_Part() start");
  _InitDisplay();
  if (hasFastPartialUpdate)
  {
    _writeCommand(0xE0); // Cascade Setting (CCSET)
    _writeData(0x02);    // TSFIX
    _writeCommand(0xE5); // Force Temperature (TSSET)
    _writeData(0x6E);    // 110
  }
  _PowerOn();
  _using_partial_mode = true;
  Serial.println("[michal FIXME] _Init_Part() end");
}

void GxEPD2_750c_ZJY::_Update_Full()
{
  // debug
  Serial.println("[michal FIXME] _Update_Full() start");
  if (useFastFullUpdate)
  {
    _writeCommand(0xE0); // Cascade Setting (CCSET)
    _writeData(0x02);    // TSFIX
    _writeCommand(0xE5); // Force Temperature (TSSET)
    _writeData(0x5A);    // 90
  }
  else
  {
    _writeCommand(0xE0); // Cascade Setting (CCSET)
    _writeData(0x00);    // no TSFIX, Temperature value is defined by internal temperature sensor
    _writeCommand(0x41); // TSE, Enable Temperature Sensor
    _writeData(0x00);    // TSE, Internal temperature sensor switch
  }
  _writeCommand(0x12); //display refresh
  _waitWhileBusy("_Update_Full", full_refresh_time);
  Serial.println("[michal FIXME] _Update_Full() end");
}

void GxEPD2_750c_ZJY::_Update_Part()
{
  Serial.println("[michal FIXME] _Update_Part() start");
  _writeCommand(0x12); //display refresh
  _waitWhileBusy("_Update_Part", partial_refresh_time);
  Serial.println("[michal FIXME] _Update_Part() end");
}
