#include <genesis.h>
#include "resources.h"

static fix32 xLookup[256]; // full range for JOY_readJoypadX()

static void calculateXLookup()
{
	// T2
	// $70-$83   :  112-131   : Total hvals 20
	// $84-$B6; $E5-$FF; $00-$42  :  132-182; 229-255; 0-66   :  51; 27; 67  -> only 51+27+66 = 145 hvals
	// $43-$6F   : 67-111  : 45 hvals
	//  there's only 210 havls total with many offscreen
	//  The active area is 290 < 320 pixels.
	// since I'm going to bother with a calibration step  and offset I will use arbitray values
	//  in the lookup
	//
	//  My own experience has puts 84  the left edge of the monitor, so I'll start with 60
	fix32 pos = FIX32(-40);
	for (int i = 60; i < 183; ++i)
	{
		xLookup[i] = pos;
		pos = fix32Add(pos, FIX32(2));
	}
	for (int i = 229; i < 256; ++i)
	{
		xLookup[i] = pos;
		pos = fix32Add(pos, FIX32(2));
	}
	for (int i = 0; i < 60; ++i)
	{
		xLookup[i] = pos;
		pos = fix32Add(pos, FIX32(2));
	}
}

int main()
{
	///////////////////////////////////////////////////////////////////////////////////
	// Sprite Setup
	Sprite *targetSprite = NULL;
	VDP_setPalette(PAL1, target.palette->data);
	VDP_setPalette(PAL2, crosshairs.palette->data);
	SPR_init();

	// crosshair is 16x16
	Sprite *crosshairsSprite = NULL;
	fix32 crosshairsPosX = FIX32(152.0);
	fix32 crosshairsPosY = FIX32(104.0);
	crosshairsSprite = SPR_addSprite(&crosshairs,								 // Sprite defined in resources
																	 fix32ToInt(crosshairsPosX), // starting X position
																	 fix32ToInt(crosshairsPosY), // starting Y position
																	 TILE_ATTR(PAL2,						 // specify palette
																						 1,								 // Tile priority ( with background)
																						 FALSE,						 // flip the sprite vertically?
																						 FALSE						 // flip the sprite horizontally
																						 ));

	// set target in the center position (target is 32x32 )
	fix32 targetPosX = FIX32(144.0); // 160 - 16 = 144
	fix32 targetPosY = FIX32(96.0);	 // 112 - 16 = 96

	targetSprite = SPR_addSprite(&target,								 // Sprite defined in resource.res
															 fix32ToInt(targetPosX), // starting X position
															 fix32ToInt(targetPosY), // starting Y position
															 TILE_ATTR(PAL1,				 // specify palette
																				 1,						 // Tile priority ( with background)
																				 FALSE,				 // flip the sprite vertically?
																				 FALSE				 // flip the sprite horizontally
																				 ));

	///////////////////////////////////////////////////////////////////////////////////
	// Menacer Setup
	// create lookup table
	calculateXLookup();

	// Set background brighter than 0.	Black background
	// prevents menacer from returning X, Y values.
	VDP_setPaletteColor(15, 0x0000);
	VDP_setTextPalette(0);
	VDP_setPaletteColor(0, 0x0844);

	// check Port 2 for the Sega Menacer
	bool menacerFound = FALSE;
	u8 portType = JOY_getPortType(PORT_2);
	if (portType == PORT_TYPE_MENACER)
	{
		JOY_setSupport(PORT_2, JOY_SUPPORT_MENACER);
		menacerFound = TRUE;
		VDP_drawText("Menacer FOUND!", 13, 1);
	}
	else
	{
		VDP_drawText("Menacer NOT found.", 11, 1);
	}

	VDP_drawText("A:", 18, 5);
	VDP_drawText("B:", 18, 6);
	VDP_drawText("C:", 18, 7);
	VDP_drawText("START:", 14, 8);
	///////////////////////////////////////////////////////////////////////////////////
	// Main Loop!
	while (TRUE)
	{
		if (menacerFound)
		{
			// get the button states
			u16 value = JOY_readJoypad(JOY_2);
			if (value & BUTTON_A)
			{
				VDP_drawText("ON", 21, 5);
			}
			else
			{
				VDP_drawText("  ", 21, 5);
			}

			if (value & BUTTON_B)
			{
				VDP_drawText("ON", 21, 6);
			}
			else
			{
				VDP_drawText("  ", 21, 6);
			}

			if (value & BUTTON_C)
			{
				VDP_drawText("ON", 21, 7);
			}
			else
			{
				VDP_drawText("  ", 21, 7);
			}

			if (value & BUTTON_START)
			{
				VDP_drawText("ON", 21, 8);
			}
			else
			{
				VDP_drawText("  ", 21, 8);
			}

			// The menacer appears to return 8-bit values (0 to 255)
			// if both values are -1, the gun is aiming off screen.
			s16 xVal = JOY_readJoypadX(JOY_2);
			s16 yVal = JOY_readJoypadY(JOY_2);
			char message[40];
			sprintf(message, "Menacer Values x:%d, y:%d      ", xVal, yVal);
			VDP_drawText(message, 8, 3);

			//  Render with lookup table
			crosshairsPosX = fix32Sub(xLookup[xVal], FIX32(8));
			crosshairsPosY = fix32Sub(FIX32(yVal), FIX32(8));
		}

		// Set the Sprite Positions.
		SPR_setPosition(crosshairsSprite, fix32ToInt(crosshairsPosX), fix32ToInt(crosshairsPosY));
		SPR_update();
		SYS_doVBlankProcess();
	}
}
