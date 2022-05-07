/*
 * MonoLcd.h
 *
 *  Created on: 6 May 2022
 *      Author: David
 */

#ifndef SRC_DISPLAY_LCD_MONOLCD_H_
#define SRC_DISPLAY_LCD_MONOLCD_H_

#include "Lcd.h"

class MonoLcd : public Lcd
{
public:
	MonoLcd(PixelNumber nr, PixelNumber nc, const LcdFont * const fnts[], size_t nFonts, SpiMode mode) noexcept;
	virtual ~MonoLcd();

	// Get the SPI frequency
	uint32_t GetSpiFrequency() const noexcept override final { return device.GetFrequency(); }

	// Initialize the display
	void Init(Pin p_csPin, Pin p_a0Pin, bool csPolarity, uint32_t freq, uint8_t p_contrastRatio, uint8_t p_resistorRatio) noexcept override final;

	// Clear part of the display and select non-inverted text.
	void Clear(PixelNumber top, PixelNumber left, PixelNumber bottom, PixelNumber right) noexcept override final;

	// Set, clear or invert a pixel
	//  x = x-coordinate of the pixel, measured from left hand edge of the display
	//  y = y-coordinate of the pixel, measured down from the top of the display
	//  mode = whether we want to set or clear the pixel
	void SetPixel(PixelNumber y, PixelNumber x, bool mode) noexcept override final;

	// Draw a bitmap
	//  x0 = x-coordinate of the top left, measured from left hand edge of the display. Currently, must be a multiple of 8.
	//  y0 = y-coordinate of the top left, measured down from the top of the display
	//  width = width of bitmap in pixels. Currently, must be a multiple of 8.
	//  rows = height of bitmap in pixels
	//  data = bitmap image, must be ((width/8) * rows) bytes long
	void BitmapImage(PixelNumber top, PixelNumber left, PixelNumber height, PixelNumber width, const uint8_t data[]) noexcept override final;

	// Draw a bitmap row
	//  x0 = x-coordinate of the top left, measured from left hand edge of the display
	//  y0 = y-coordinate of the top left, measured down from the top of the display
	//  width = width of bitmap in pixels
	//  data = bitmap image, must be ((width + 7)/8) bytes long
	void BitmapRow(PixelNumber top, PixelNumber left, PixelNumber width, const uint8_t data[], bool invert) noexcept override final;

	// Set the foreground colour. Does nothing on monochrome displays.
	void SetForegroundColour(Colour col) noexcept override final { }

	// Set the background colour. Does nothing on monochrome displays.
	void SetBackgroundColour(Colour col) noexcept override final { }

protected:
	virtual void HardwareInit() noexcept = 0;

	// Write one column of character data at (row, column)
	void WriteColumnData(uint16_t columnData, uint8_t ySize) noexcept override final;

	void SetDirty(PixelNumber r, PixelNumber c) noexcept;
	void SetRectDirty(PixelNumber top, PixelNumber left, PixelNumber bottom, PixelNumber right) noexcept;

	size_t imageSize;
	uint8_t *image;									// image buffer
	SharedSpiClient device;

	Pin csPin;
	Pin a0Pin;
	uint8_t contrastRatio;
	uint8_t resistorRatio;
	PixelNumber startRow, startCol, endRow, endCol;	// coordinates of the dirty rectangle
	PixelNumber nextFlushRow;						// which row we need to flush next
};

#endif /* SRC_DISPLAY_LCD_MONOLCD_H_ */
