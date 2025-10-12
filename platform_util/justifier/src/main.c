#include <genesis.h>
#include "resources.h"


static fix32 xLookup[256]; // full range for JOY_readJoypadX()

static void calculateXLookup() {
  // My blue justifier appears to return 34 through 176 when I pan 
	// across my TV screen in H32 mode.  So about 142 values.

  fix32 pos = FIX32(0);
  for( int i=34; i < 176; ++i ) {
    xLookup[i] =  pos;
    pos = fix32Add( pos, FIX32(2.25) );
  }

}


int main()
{

	//////////////////////////////////////////////////////////////
	// Setup background B
	PAL_setPalette(PAL0, bg_pal.data, CPU);
	int ind = TILE_USER_INDEX;
	VDP_drawImageEx(BG_B, &bg, TILE_ATTR_FULL(PAL0, FALSE, FALSE, FALSE, ind), 0, 0, FALSE, TRUE);


	///////////////////////////////////////////////////////////////////////////////////
	// Sprite Setup
	Sprite *targetSprite = NULL;
	PAL_setPalette(PAL1, target_pal.data, CPU);
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
	// Justifier Setup
	// create lookup table
	calculateXLookup();	

	// Set background brighter than 0.	darker backgrounds
	// prevent Justifier from returning X, Y values.
	PAL_setColor(15, 0x0000);
	VDP_setTextPalette(0);
	PAL_setColor(0, RGB24_TO_VDPCOLOR(0x4488ff)); // seems to work OK

	// check Port 2 for the Konami Justifier
	bool justifierFound = FALSE;
	u8 portType = JOY_getPortType(PORT_2);
	if(portType == PORT_TYPE_JUSTIFIER )
	{
		JOY_setSupport(PORT_2, JOY_SUPPORT_JUSTIFIER_BLUE );
		justifierFound = TRUE;
		VDP_drawText("Justifier FOUND!", 11, 2);
	} else {
		VDP_drawText("Justifier NOT found.", 10, 2);
	}

	///////////////////////////////////////////////////////////////////////////////////
	// Draw text
	char message[40];
	// use intToStr() to print row numbers
	for( s32 i=0; i < 28; ++i ) {
		intToStr( i, message, 1 );
		VDP_drawText( message, 0, i );
	}

	// use uintToStr() to print column numbers
	for( u32 i=0; i < 40; ++i ) {
		u32 tmp = i%10;
		uintToStr( tmp, message, 1 );
		// draw ones place
		VDP_drawText( message, i, 0 );
		// draw tens place
		if( i > 0 ) {
			if( tmp == 0 ) {
				uintToStr( i/10, message, 1 );
				VDP_drawText( message, i, 1 );
			}
		}
	}


	///////////////////////////////////////////////////////////////////////////////////
	// Main Loop!
	while (TRUE)
	{
		if( justifierFound ) {

			// get the Justifier trigger state
			u16 value = JOY_readJoypad(JOY_2);
			if (value & BUTTON_A)
			{
				VDP_drawText("A: ON", 18, 9);
			}
			else
			{
				VDP_drawText("A:   ", 18, 9);
			}
  
  
			// My blue justifier appears to return 34 through 176 when I use it on 
			// H32 mode.
			// if both values are -1, the gun is aiming off screen.
			s16 xVal = JOY_readJoypadX(JOY_2);
			s16 yVal = JOY_readJoypadY(JOY_2);
			char message[40];
			sprintf(message, "Justifier Values x:%d, y:%d      ", xVal, yVal);
			VDP_drawText(message, 7, 7);
  
			// Render with lookup table
			crosshairsPosX = fix32Sub(xLookup[xVal], FIX32(8));
			crosshairsPosY = fix32Sub(FIX32(yVal), FIX32(8));
		}

		// Set the Sprite Positions.
		SPR_setPosition(crosshairsSprite, fix32ToInt(crosshairsPosX), fix32ToInt(crosshairsPosY));
		SPR_update();

		SYS_doVBlankProcess(); 
	}
}
