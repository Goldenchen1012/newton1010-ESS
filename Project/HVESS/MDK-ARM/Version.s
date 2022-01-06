;*******************************************************************************
;* File Name          : startup_stm32l496xx.s
;* Author             : MCD Application Team
;* Description        : STM32L496xx Ultra Low Power devices vector table for MDK-ARM toolchain.
;*                      This module performs:
;*                      - Set the initial SP
;*                      - Set the initial PC == Reset_Handler
;*                      - Set the vector table entries with the exceptions ISR address
;*                      - Branches to __main in the C library (which eventually
;*                        calls main()).
;*                      After Reset the Cortex-M4 processor is in Thread mode,
;*                      priority is Privileged, and the Stack is set to Main.
;* <<< Use Configuration Wizard in Context Menu >>>
;*******************************************************************************
;*
;* <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
;* All rights reserved.</center></h2>
;*
;* This software component is licensed by ST under Apache License, Version 2.0,
;* the "License"; You may not use this file except in compliance with the
;* License. You may obtain a copy of the License at:
;*                        opensource.org/licenses/Apache-2.0
;*
;*******************************************************************************
;

                PRESERVE8
                THUMB
;wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww

	IF      :DEF:__USE_BOOTLOADER

			AREA |.ARM.__AT_0x08004000|, CODE, READONLY 

					;	"0123456789ABCDEF"
				DCB		"SmpHvEssFirmware"	;0~15	檔頭識別碼1
				DCD		0x00000001			;16~19	version
				DCD		0					;20~23	length	固定為0,由更新程式填寫
				DCD		0x20211202			;24~27	date
				DCD		0x00174500			;28~31	time
				DCD		0					;32~35	checksum,初始值固定為0
				DCD		0					;36~39
				DCD		0					;40~43
				DCD		0					;44~47
				DCB		"SMPhvessFirmWare"	;48~63	檔頭識別碼2
				DCB		"DaVinCi         "	;專案識別碼
				;DCB		__DATE__
				
	ENDIF
;wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww



                 END

;************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE*****
