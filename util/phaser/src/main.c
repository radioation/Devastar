#include <genesis.h>
#include "resources.h"

u16 crosshairsMode = 0; // 0 raw values, 1 use X lookup
static fix32 xLookup[256]; // full range for JOY_readJoypadX()

static void calculateXLookup()
{
	// My SMS Phaser appears to return 30 through 180 when I pan
	// across my TV screen in H32 mode.   so about 150 values over 320 pixels
	fix32 pos = FIX32(0);
	for (int i = 30; i < 180; ++i)
	{
		xLookup[i] = pos;
		pos = fix32Add(pos, FIX32(2.13333));
	}
}

static void joypadHandler(u16 joypadId, u16 changed, u16 joypadState)
{
	// standard controller handles modes
	if (joypadId == JOY_1)
	{
		if (changed == BUTTON_A && joypadState == BUTTON_A)
		{
			++crosshairsMode;
			if (crosshairsMode > 1)
			{
				crosshairsMode = 0;
			}
		}
	}
}

int main()
{

	///////////////////////////////////////////////////////////////////////////////////
	// Sprite Setup
	Sprite *targetSprite = NULL;
	VDP_setPalette(PAL1, target.palette->data);
	SPR_init();

	// crosshair is 16x16
	Sprite *crosshairsSprite = NULL;
	fix32 crosshairsPosX = FIX32(152.0);
	fix32 crosshairsPosY = FIX32(104.0);
	crosshairsSprite = SPR_addSprite(&crosshairs,								 // Sprite defined in resources
																	 fix32ToInt(crosshairsPosX), // starting X position
																	 fix32ToInt(crosshairsPosY), // starting Y position
																	 TILE_ATTR(PAL1,						 // specify palette
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
	SPR_setPosition( targetSprite, fix32ToInt( targetPosX ), fix32ToInt( targetPosY ) );

	///////////////////////////////////////////////////////////////////////////////////
	// Phaser Setup
	// create lookup table
	calculateXLookup();	

	// Set background brighter than 0.	darker backgrounds
	// prevent Phaser from returning X, Y values.
	VDP_setPaletteColor(15, 0x0000);
	VDP_setTextPalette(0);
	VDP_setPaletteColor(0, RGB24_TO_VDPCOLOR(0x44aaff)); // seems to work OK

	// Can't check for phaser with JOY_getPortType().  Just assume we've got a Phaser attached to Port 2
	JOY_setSupport(PORT_2, JOY_SUPPORT_PHASER);

	VDP_drawText("Press A to change drawing mode", 5, 5);

	// Asynchronous joystick handler.
	JOY_setEventHandler(joypadHandler);

	///////////////////////////////////////////////////////////////////////////////////
	// Main Loop!
	while (TRUE)
	{
		// get the Phaser trigger state
		u16 value = JOY_readJoypad(JOY_2);
		if (value & BUTTON_A)
		{
			VDP_drawText("A: ON", 18, 9);
		}
		else
		{
			VDP_drawText("A:   ", 18, 9);
		}

		// H32 mode.
		// if both values are -1, the gun is aiming off screen.
		s16 xVal = JOY_readJoypadX(JOY_2);
		s16 yVal = JOY_readJoypadY(JOY_2);
		char message[40];
		sprintf(message, "Phaser Values x:%d, y:%d      ", xVal, yVal);
		VDP_drawText(message, 7, 7);

		// set crosshairs position. Subtract 8 from each to compensate for 16x16 sprite
		switch (crosshairsMode)
		{
		case 0: // raw
			VDP_drawText("   Render raw joypad values   ", 5, 5);
			crosshairsPosX = fix32Mul(FIX32(xVal), FIX32(2.52));
			crosshairsPosY = FIX32(yVal - 8);
			break;
		case 1: // with lookup
			VDP_drawText("   Render with lookup table   ", 5, 5);
			crosshairsPosX = fix32Sub(xLookup[xVal], FIX32(8));
			crosshairsPosY = fix32Sub(FIX32(yVal), FIX32(8));
			break;
		default:
			break;
		}

		// Set the Sprite Positions.
		SPR_setPosition(crosshairsSprite, fix32ToInt(crosshairsPosX), fix32ToInt(crosshairsPosY));
		SPR_update();

		SYS_doVBlankProcess(); 
	}
}