/**
  ******************************************************************************
  * @file    smp_MX25L_Driver.c
  * @author  John Chen/ Golden
  * @version V0.0.2
  * @date    2022/01/03
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 SMP</center></h2>
  *
  * 
  *
  ******************************************************************************
  */
#include "smp_MX25L_Driver.h"
#include "Bsp.h"
#include "smp_debug.h"
#include "smp_gpio.h"
#include "smp_fifo_flash.h"
#include "LibSwTimer.h"

#include <string.h>	

#if 0
#define MX25LXXX_DEBUG_PRINTF
#endif

/* MX25L Instructions ****************************************************************************/
/*      Command                                Value      Description               Addr   Data  */
/*                                                                                  Dummy        */
#define MX25L_READ                             0x03    /* Read data bytes          3   0   >=1   */
#define MX25L_FAST_READ                        0x0b    /* Higher speed read        3   1   >=1   */
#define MX25L_2READ                            0xbb    /* 2 x I/O read command     */
#define MX25L_DREAD                            0x3b    /* 1I / 2O read command     3   1   >=1   */
#define MX25L_4READ                            0xeb    /* 4 x I/O read command     */
#define MX25L_QREAD                            0x6b    /* 1I / 4O read command     3   1   >=1   */
#define MX25L_WREN                             0x06    /* Write Enable             0   0   0     */ 
#define MX25L_WRDI                             0x04    /* Write Disable            0   0   0     */ 
#define MX25L_RDSR                             0x05    /* Read status register     0   0   >=1   */ 
#define MX25L_RDCR                             0x15    /* Read config register     0   0   >=1   */ 
#define MX25L_WRSR                             0x01    /* Write stat/conf register 0   0   2     */ 
#define MX25L_4PP                              0x38    /* Quad page program        3   0   1-256 */
#define MX25L_SE                               0x20    /* 4Kb Sector erase         3   0   0     */
#define MX25L_BE32                             0x52    /* 32Kbit block Erase       3   0   0     */
#define MX25L_BE64                             0xd8    /* 64Kbit block Erase       3   0   0     */
#define MX25L_CE                               0xc7    /* Chip erase               0   0   0     */ 
#define MX25L_CE_ALT                           0x60    /* Chip erase (alternate)   0   0   0     */ 
#define MX25L_PP                               0x02    /* Page program             3   0   1-256 */
#define MX25L_DP                               0xb9    /* Deep power down          0   0   0     */ 
#define MX25L_RDP                              0xab    /* Release deep power down  0   0   0     */ 
#define MX25L_PGM_SUSPEND                      0x75    /* Suspends program         0   0   0     */ 
#define MX25L_ERS_SUSPEND                      0xb0    /* Suspends erase           0   0   0     */ 
#define MX25L_PGM_RESUME                       0x7A    /* Resume program           0   0   0     */ 
#define MX25L_ERS_RESUME                       0x30    /* Resume erase             0   0   0     */ 
#define MX25L_RDID                             0x9f    /* Read identification      0   0   3     */ 
#define MX25L_RES                              0xab    /* Read electronic ID       0   3   1     */
#define MX25L_REMS                             0x90    /* Read manufacture and ID  1   2   >=2   */
#define MX25L_ENSO                             0xb1    /* Enter secured OTP        0   0   0     */ 
#define MX25L_EXSO                             0xc1    /* Exit secured OTP         0   0   0     */ 
#define MX25L_RDSCUR                           0x2b    /* Read security register   0   0   0     */ 
#define MX25L_WRSCUR                           0x2f    /* Write security register  0   0   0     */ 
#define MX25L_RSTEN                            0x66    /* Reset Enable             0   0   0     */ 
#define MX25L_RST                              0x99    /* Reset Memory             0   0   0     */ 
#define MX25L_RDSFDP                           0x5a    /* read out until CS# high  */
#define MX25L_SBL                              0xc0    /* Set Burst Length         */
#define MX25L_SBL_ALT                          0x77    /* Set Burst Length         */
#define MX25L_NOP                              0x00    /* No Operation             0   0   0     */ 
#define MX25L_DUMMY                            0xff

#define MX25L_MANUFACTURER_ID                  0xc2  /* Macronix manufacturer ID */
#define MX25L_MEMORY_TYPE_ID                   0x20  /* MX25Lx  memory type */
#define MX25L_MEMORY_DENSITY_ID                0x17  /* MX25L6433F memory capacity */

#define MX25L_SR_WIP                           (1 << 0)      /* Bit 0: Write in progress */
#define MX25L_SR_WEL                           (1 << 1)      /* Bit 1: Write enable latch */      
#define MX25L_SR_BP                            (0b1111 << 2) /* Bits 2-5: Block protect bits */
#define MX25L_SR_QE                            (1 << 6)      /* Bit 6: Quad enable */
#define MX25L_SR_SRWD                          (1 << 7)      /* Bit 7: Status register write protect */

#define FLASH_READ_BUFFER_SIZE 256

#define MX25L_FLAH_BUFFER		{                                                                 \
                                .buffers.rx_buf             = flash_read_buffer,              \
                                .buffers.rx_buf_size        = FLASH_READ_BUFFER_SIZE-50          \
                            }  

smp_gpio_t MX25L_write_protection_handler;
smp_gpio_t MX25L_hold_handler;
smp_gpio_t MX25L_PIN;
smp_spi_cs_t MX25L_CS0;
smp_spi_t MX25L_SPI_1;

uint8_t MX25L_tx_data[BSP_SPI2_TX_BUFFER_SIZE+50];
uint8_t MX25L_rx_data[BSP_SPI2_RX_BUFFER_SIZE+50];

smp_fifo_flash_t flash_read_fifo = {0};

smp_flash_package flash_read_buffer[FLASH_READ_BUFFER_SIZE];
smp_flash_t mDavinci_flash = MX25L_FLAH_BUFFER;
void MX25L_SPI_1_event_handler(smp_spi_evt_type p_evt);
bool UseDMAFlag = false;
			
int8_t smp_mx25l_flash_read_status(smp_mx25l_status *mx251_status);
uint8_t smp_mx25l_is_flash_spi_Ready(void);

static void spirom_event_handler(smp_flash_evt_type p_evt)
{
	switch(p_evt){
		case SMP_FLASH_EVENT_READ_DONE:

		break;
		case SMP_FLASH_EVENT_WRITE_DONE:

		break;
		case SMP_FLASH_EVENT_ERASE_DONE:
		
		break;
		default:
		break;
	
	}

}

void appSerialCanDavinciSendTextMessage(char *msg);
#define	mx25DbgMsg(str)	appSerialCanDavinciSendTextMessage(str)

static void smp_mx25l_flash_SwTimerHandler(__far void *dest, uint16_t evt, void *vDataPtr)
{
//	char	str[100];
//	static uint8_t	count = 0;
	static uint32_t	addr =0 ;
	uint16_t	sector,i;
	uint8_t		buffer[256];
	char	str[100];
	
	smp_mx25l_status mx251_status;
	
	//GPIOD->ODR |= GPIO_PIN_14;
	if(evt == LIB_SW_TIMER_EVT_SW_10MS_2)
	{
		GPIOD->ODR |= GPIO_PIN_15;
		if(smp_mx25l_is_flash_spi_Ready()&& MX25L_SPI_get_command_size()>0)
		{
			smp_mx25l_flash_read_status(&mx251_status);
			if((mx251_status.status1&STATUS_WRITE_IN_PROGRESS)==0)
			{
				MX25L_SPI_send_command();
			}
		}
//		GPIOD->ODR &= ~GPIO_PIN_15;
	}
	else if(evt == LIB_SW_TIMER_EVT_SW_1S)
	{
#if 0		
		if((addr&0xfff) == 0)
		{
			sector = addr / 4096;
			smp_mx25l_flash_sector_erase_sectornum(sector , spirom_event_handler);
			//sprintf(str,"
			mx25DbgMsg("Erase");
		}
		for(i=0; i<256; i++)
			buffer[i] = i;
		smp_mx25l_flash_page_program(addr/256, buffer, 256,spirom_event_handler);
		addr += 256L;
		mx25DbgMsg("Write");
#endif		
	}
}
		
int8_t flash_fifo_init(smp_fifo_flash_t * p_fifo, smp_flash_package * p_buf, uint16_t buf_size)
{
    // Check buffer for null pointer.
    if (p_buf == NULL){
        return SMP_ERROR_NULL;
    }
    p_fifo->buffer_addr   = (smp_flash_package *)p_buf;
    p_fifo->buffer_size 	= buf_size - 1;
    p_fifo->in      			= 0;
    p_fifo->out     			= 0;

    return SMP_SUCCESS;
}

int8_t smp_mx25l_flash_init(void)
{
	//initial MX25L write protection pin
	MX25L_write_protection_handler.port = BSP_MX25L_WRITE_PROTECTON_GPIO_PORT;
	MX25L_write_protection_handler.pin = BSP_MX25L_WRITE_PROTECTON_PIN;
	MX25L_write_protection_handler.mode = SMP_GPIO_MODE_OUTPUT_PP;	
	
	#ifdef MX25LXXX_DEBUG_PRINTF 
	printf("flash init\r\n");
	#endif 
	
	if(smp_gpio_init(&MX25L_write_protection_handler) != HAL_OK){
		return SMP_ERROR_NOT_FOUND;
	}
	smp_gpio_set_state(&MX25L_write_protection_handler, GPIO_ACTIVE_HIGH);

	MX25L_hold_handler.port = BSP_MX25L_HOLD_GPIO_PORT;
	MX25L_hold_handler.pin = BSP_MX25L_HOLD_PIN;
	MX25L_hold_handler.mode = SMP_GPIO_MODE_OUTPUT_PP;	
	
	if(smp_gpio_init(&MX25L_hold_handler) != HAL_OK){
		return SMP_ERROR_NOT_FOUND;
	}
	smp_gpio_set_state(&MX25L_hold_handler, GPIO_ACTIVE_HIGH);

	
	// CS Initial
	MX25L_PIN.port = BSP_MX25L_CS_GPIO_PORT;
	MX25L_PIN.pin = BSP_MX25L_CS_PIN;	
	MX25L_CS0.spi_num = SPI_module3;
	MX25L_CS0.cs_handler = MX25L_PIN;	
	smp_spi_master_cs_init(&MX25L_CS0);
	
	MX25L_SPI_1.num = SPI_module3;
	MX25L_SPI_1.mode = SPI_mode0;
	if(smp_spi_master_init(&MX25L_SPI_1, MX25L_SPI_1_event_handler, false) != SMP_SUCCESS){
		return SMP_ERROR_NOT_FOUND;
	}
	// Configure buffer RX buffer.
	flash_fifo_init(&flash_read_fifo, mDavinci_flash.buffers.rx_buf, mDavinci_flash.buffers.rx_buf_size);
	smp_fifo_flash_open(&flash_read_fifo);
	
	LibSwTimerOpen(smp_mx25l_flash_SwTimerHandler, 0);

	return SMP_SUCCESS;	
}

int8_t smp_mx25l_flash_write_enable(void)
{
  /* Send "Write Enable (WREN)" command */
	int8_t tempstate;
	MX25L_tx_data[0] = MX25L_WREN;
	UseDMAFlag = false;
	tempstate = smp_spi_master_send_recv_blocking(&MX25L_SPI_1, MX25L_tx_data , 1 , 0, 0 , &MX25L_CS0);
  return tempstate;
}

int8_t smp_mx25l_flash_write_disable(void)
{
  /* Send "Write Disable (WRDI)" command */
	int8_t tempstate;
	MX25L_tx_data[0] = MX25L_WRDI;
	UseDMAFlag = false;
	tempstate = smp_spi_master_send_recv_blocking(&MX25L_SPI_1, MX25L_tx_data , 1 , 0, 0 , &MX25L_CS0);
  return tempstate;
}

int8_t smp_mx25l_flash_read_ID(smp_mx25l_ID *mx251_ID)
{
  /* Send "read ID (RDID)" command */
	int8_t tempstate;
	MX25L_tx_data[0] = MX25L_RDID;
	UseDMAFlag = false;
	tempstate = smp_spi_master_send_recv_blocking(&MX25L_SPI_1, MX25L_tx_data , 1 , MX25L_rx_data, 3 , &MX25L_CS0);
	mx251_ID->Manufacture_ID = MX25L_rx_data[0];
	mx251_ID->Device_ID = (MX25L_rx_data[1] << 8) + MX25L_rx_data[2];
  return tempstate;
}

uint8_t smp_spi_master_is_spi_ready(smp_spi_t *spi);

uint8_t smp_mx25l_is_flash_spi_Ready(void)
{
	return smp_spi_master_is_spi_ready(&MX25L_SPI_1);
}

int8_t smp_mx25l_flash_read_status(smp_mx25l_status *mx251_status)
{
  /* Send "read status (RDSR)" command */
	int8_t tempstate;
	MX25L_tx_data[0] = MX25L_RDSR;
	UseDMAFlag = false;
	tempstate = smp_spi_master_send_recv_blocking(&MX25L_SPI_1, MX25L_tx_data , 1 , MX25L_rx_data, 2 , &MX25L_CS0);
	mx251_status->status1 = MX25L_rx_data[0];
	mx251_status->status2 = MX25L_rx_data[1];
  return tempstate;
}

int8_t smp_mx25l_flash_read_configuration(smp_mx25l_config *mx251_config)
{
  /* Send "read config (RDCR)" command */
	int8_t tempstate;
	MX25L_tx_data[0] = MX25L_RDCR;
	UseDMAFlag = false;
	tempstate = smp_spi_master_send_recv_blocking(&MX25L_SPI_1, MX25L_tx_data , 1 , MX25L_rx_data, 2 , &MX25L_CS0);
	mx251_config->config1 = MX25L_rx_data[0];
	mx251_config->config2 = MX25L_rx_data[1];
  return tempstate;
}

int8_t smp_mx25l_flash_write_status(smp_mx25l_status *mx251_status,smp_mx25l_config *mx251_config)
{
  /* Send "write status (WRSR)" command */
	int8_t tempstate;
	MX25L_tx_data[0] = MX25L_WRSR;
	MX25L_tx_data[1] = mx251_status->status1;
	MX25L_tx_data[2] = mx251_config->config1;
	UseDMAFlag = false;
	smp_mx25l_flash_write_enable();
	tempstate = smp_spi_master_send_recv_blocking(&MX25L_SPI_1, MX25L_tx_data , 3 , 0, 0 , &MX25L_CS0);
  return tempstate;
}


int8_t smp_mx25l_flash_read_data_bytes(uint8_t *flash_addr, uint8_t *buffer, uint16_t read_byte_num, smp_flash_event_t smp_flash_event_handle)
{
  /* Send "read data bytes (READ)" command */
	smp_flash_package	FlashPkg;
	FlashPkg.command = MX25L_READ;
	memcpy(FlashPkg.addr, flash_addr, 3);
	FlashPkg.read_buffer = buffer;
	FlashPkg.Dummy = 0;
	FlashPkg.R_W_bytes = read_byte_num;
	FlashPkg.flash_callback = smp_flash_event_handle;
	if(smp_fifo_flash_push(&flash_read_fifo, &FlashPkg)== SMP_SUCCESS){
		return SMP_SUCCESS;
	}
  return SMP_ERROR_RESOURCES;
}

int8_t smp_mx25l_flash_fast_read_data_bytes_addr(uint8_t *flash_addr ,uint8_t *buffer, uint16_t read_byte_num , smp_flash_event_t smp_flash_event_handle)
{
  /* Send "read data bytes (READ)" command */
	smp_flash_package	FlashPkg;
	FlashPkg.command = MX25L_FAST_READ;
	memcpy(FlashPkg.addr, flash_addr, 3);
	FlashPkg.read_buffer = buffer;
	FlashPkg.Dummy = MX25L_DUMMY;
	FlashPkg.R_W_bytes = read_byte_num;
	FlashPkg.flash_callback = smp_flash_event_handle;
	
	#ifdef MX25LXXX_DEBUG_PRINTF 
	printf("flash read addr 0x%02x%02x%02x\r\n",FlashPkg.addr[0],FlashPkg.addr[1],FlashPkg.addr[2]);
	#endif 
	
	if(smp_fifo_flash_push(&flash_read_fifo, &FlashPkg)== SMP_SUCCESS){
		return SMP_SUCCESS;
	}
  return SMP_ERROR_RESOURCES;
}

int8_t smp_mx25l_flash_fast_read_data_bytes_page(uint16_t page , uint8_t *buffer , uint16_t read_byte_num , smp_flash_event_t smp_flash_event_handle)
{
  /* Send "read data bytes (READ)" command */
	smp_flash_package	FlashPkg;
	FlashPkg.command = MX25L_FAST_READ;
  FlashPkg.addr[0] = (uint8_t)((page << MX25L_MX25L6433F_PAGE_SHIFT) >> 16) ;
	FlashPkg.addr[1] = (uint8_t)((page << MX25L_MX25L6433F_PAGE_SHIFT) >> 8) ;
	FlashPkg.addr[2] = (uint8_t)(page << MX25L_MX25L6433F_PAGE_SHIFT);
	
	#ifdef MX25LXXX_DEBUG_PRINTF 
	printf("flash read page 0x%x\r\n",page);
	printf("flash read addr 0x%02x%02x%02x\r\n",FlashPkg.addr[0],FlashPkg.addr[1],FlashPkg.addr[2]);
	#endif
	
	FlashPkg.read_buffer = buffer;
	FlashPkg.Dummy = MX25L_DUMMY;
	FlashPkg.R_W_bytes = read_byte_num;
	FlashPkg.flash_callback = smp_flash_event_handle;
	if(smp_fifo_flash_push(&flash_read_fifo, &FlashPkg)== SMP_SUCCESS){
		return SMP_SUCCESS;
	}
  return SMP_ERROR_RESOURCES;
}

int8_t smp_mx25l_flash_fast_read_data_bytes_page_blocking(uint16_t page , uint8_t *buffer , uint16_t read_byte_num)
{
  /* Send "read data bytes (READ)" command */
	smp_flash_package	FlashPkg;
	FlashPkg.command = MX25L_FAST_READ;
  FlashPkg.addr[0] = (uint8_t)((page << MX25L_MX25L6433F_PAGE_SHIFT) >> 16) ;
	FlashPkg.addr[1] = (uint8_t)((page << MX25L_MX25L6433F_PAGE_SHIFT) >> 8) ;
	FlashPkg.addr[2] = (uint8_t)(page << MX25L_MX25L6433F_PAGE_SHIFT);
	
	FlashPkg.read_buffer = buffer;
	FlashPkg.Dummy = MX25L_DUMMY;
	FlashPkg.R_W_bytes = read_byte_num;
	return smp_spi_master_send_recv_blocking(&MX25L_SPI_1, (uint8_t* )&FlashPkg , 5 , FlashPkg.read_buffer, FlashPkg.R_W_bytes, &MX25L_CS0);

}

int8_t smp_mx25l_flash_sector_erase_addr(uint8_t *flash_addr , smp_flash_event_t smp_flash_event_handle)
{
  /* Send "sector_erase (SE)" command */
	smp_flash_package	FlashPkg;
	FlashPkg.command = MX25L_SE;
	memcpy(FlashPkg.addr, flash_addr, 3);
	FlashPkg.flash_callback = smp_flash_event_handle;
	if(smp_fifo_flash_push(&flash_read_fifo, &FlashPkg)== SMP_SUCCESS){
		return SMP_SUCCESS;
	}
  return SMP_ERROR_RESOURCES;
}

int8_t smp_mx25l_flash_sector_erase_sectornum(uint16_t sector_num , smp_flash_event_t smp_flash_event_handle)
{
  /* Send "sector_erase (SE)" command */
	smp_flash_package	FlashPkg;
	FlashPkg.command = MX25L_SE;
	FlashPkg.addr[0] = (uint8_t)((sector_num << MX25L_MX25L6433F_SECTOR_SHIFT) >> 16) ;
	FlashPkg.addr[1] = (uint8_t)((sector_num << MX25L_MX25L6433F_SECTOR_SHIFT) >> 8) ;
	FlashPkg.addr[2] = (uint8_t)(sector_num << MX25L_MX25L6433F_SECTOR_SHIFT);
	
	#ifdef MX25LXXX_DEBUG_PRINTF 
	printf("flash erase sector 0x%x\r\n",sector_num);
	printf("flash erase addr 0x%02x%02x%02x\r\n",FlashPkg.addr[0],FlashPkg.addr[1],FlashPkg.addr[2]);
	#endif
	
	FlashPkg.flash_callback = smp_flash_event_handle;
	if(smp_fifo_flash_push(&flash_read_fifo, &FlashPkg)== SMP_SUCCESS){
		return SMP_SUCCESS;
	}
  return SMP_ERROR_RESOURCES;
}

int8_t smp_mx25l_flash_block_erase(uint8_t *flash_addr , smp_flash_event_t smp_flash_event_handle)
{ 
	/* Send "block erase (BE)" command 64k bytes*/
	smp_flash_package	FlashPkg;
	FlashPkg.command = MX25L_BE64;
	memcpy(FlashPkg.addr, flash_addr, 3);
	FlashPkg.flash_callback = smp_flash_event_handle;
	if(smp_fifo_flash_push(&flash_read_fifo, &FlashPkg)== SMP_SUCCESS){
		return SMP_SUCCESS;
	}
  return SMP_ERROR_RESOURCES;
}

int8_t smp_mx25l_flash_chip_erase(smp_flash_event_t smp_flash_event_handle)
{
  /* Send "Chip erase (CE)" command */
	smp_flash_package	FlashPkg;
	FlashPkg.command = MX25L_CE;
	FlashPkg.flash_callback = smp_flash_event_handle;
	if(smp_fifo_flash_push(&flash_read_fifo, &FlashPkg)== SMP_SUCCESS){
		return SMP_SUCCESS;
	}
  return SMP_ERROR_RESOURCES;
}

int8_t smp_mx25l_flash_page_program(uint16_t page,uint8_t *buffer,uint16_t write_byte_num,smp_flash_event_t smp_flash_event_handle)
{
  /* Send "page program (PP)" command */
	uint32_t addr = 0;
	smp_flash_package	FlashPkg;
	FlashPkg.command = MX25L_PP;
	addr = page << MX25L_MX25L6433F_PAGE_SHIFT;
	if(addr > 0x7FFF00){
		return SMP_ERROR_INVALID_PARAM;
	}
	FlashPkg.addr[0] = (uint8_t)(addr >> 16) ;
	FlashPkg.addr[1] = (uint8_t)(addr >> 8) ;
	FlashPkg.addr[2] = (uint8_t)addr;
	memcpy(FlashPkg.page_buffer, buffer, 256);
	
	#ifdef MX25LXXX_DEBUG_PRINTF 
	printf("flash wr page 0x%x\r\n",page);
	printf("flash wr addr 0x%02x%02x%02x\r\n",FlashPkg.addr[0],FlashPkg.addr[1],FlashPkg.addr[2]);
	#endif
	
	FlashPkg.Dummy = 0;
	FlashPkg.R_W_bytes = write_byte_num;
	FlashPkg.flash_callback = smp_flash_event_handle;
	if(smp_fifo_flash_push(&flash_read_fifo, &FlashPkg)== SMP_SUCCESS){
		return SMP_SUCCESS;
	}
  return SMP_ERROR_RESOURCES;
}

int8_t smp_mx25l_flash_deep_power_down(void)
{
  /* Send "Deep power-down (DP)" command */
	int8_t tempstate;
	MX25L_tx_data[0] = MX25L_DP;
	UseDMAFlag = false;
	tempstate = smp_spi_master_send_recv_blocking(&MX25L_SPI_1, MX25L_tx_data , 1 , 0, 0 , &MX25L_CS0);
  return tempstate;
}

int8_t smp_mx25l_flash_release_deep_power_down(void)
{
  /* Send "Release from Deep Power-down (RDP)" command */
	int8_t tempstate;
	MX25L_tx_data[0] = MX25L_RDP;
	UseDMAFlag = false;
	tempstate = smp_spi_master_send_recv_blocking(&MX25L_SPI_1, MX25L_tx_data , 1 , 0, 0 , &MX25L_CS0);
  return tempstate;
}

int8_t smp_mx25l_flash_reset(void)
{
  /* Send "Reset-Enable (RSTEN) and Reset (RST)" command */
	int8_t tempstate;
	MX25L_tx_data[0] = MX25L_RSTEN;
	MX25L_tx_data[1] = MX25L_RST;
	UseDMAFlag = false;
	tempstate = smp_spi_master_send_recv_blocking(&MX25L_SPI_1, MX25L_tx_data , 2 , 0, 0 , &MX25L_CS0);
  return tempstate;
}

uint16_t MX25L_SPI_get_command_size(void)
{
	uint16_t size = 0;
	smp_fifo_flash_get_size(&flash_read_fifo,&size);
	return size;
}

void MX25L_SPI_send_command(void)
{
//#define	SPI_SEND_CMD_DEBUG	
	smp_flash_package	FlashPkg;
	uint16_t size = 0;
#ifdef SPI_SEND_CMD_DEBUG	
	char	str[100];
#endif	
	smp_fifo_flash_get_size(&flash_read_fifo,&size);
	if(size > 0){
		smp_mx25l_flash_write_enable();
		smp_fifo_flash_read(&flash_read_fifo, &FlashPkg);
		MX25L_tx_data[0] = FlashPkg.command;
		memcpy(&MX25L_tx_data[1], FlashPkg.addr, 3);
		UseDMAFlag = true;
#ifdef SPI_SEND_CMD_DEBUG		
		str[0] = 0;
#endif		
		if(FlashPkg.command == MX25L_READ){
			smp_spi_master_send_recv(&MX25L_SPI_1, (uint8_t* )&FlashPkg , 4 , FlashPkg.read_buffer, FlashPkg.R_W_bytes, &MX25L_CS0);
#ifdef SPI_SEND_CMD_DEBUG			
			sprintf(str,"SPI Read:%.2X %.2X %.2X",
						FlashPkg.addr[0],
						FlashPkg.addr[1],
						FlashPkg.addr[2]);
#endif
		}else if(FlashPkg.command == MX25L_FAST_READ){
			smp_spi_master_send_recv(&MX25L_SPI_1, (uint8_t* )&FlashPkg , 5 , FlashPkg.read_buffer, FlashPkg.R_W_bytes, &MX25L_CS0);
#ifdef SPI_SEND_CMD_DEBUG			
			sprintf(str,"SPI Fast Read:%.2X %.2X %.2X",
						FlashPkg.addr[0],
						FlashPkg.addr[1],
						FlashPkg.addr[2]);
#endif			
		}else if(FlashPkg.command == MX25L_PP){
			memcpy(&MX25L_tx_data[4], FlashPkg.page_buffer, 256);
			smp_spi_master_send_recv(&MX25L_SPI_1, MX25L_tx_data , FlashPkg.R_W_bytes + 4 , 0, 0, &MX25L_CS0);
#ifdef SPI_SEND_CMD_DEBUG			
			sprintf(str,"SPI Write:%.2X %.2X %.2X",
						FlashPkg.addr[0],
						FlashPkg.addr[1],
						FlashPkg.addr[2]);
#endif			
		}else if(FlashPkg.command == MX25L_SE){
			smp_spi_master_send_recv(&MX25L_SPI_1, (uint8_t *)&FlashPkg , 4 , 0, 0, &MX25L_CS0);
#ifdef SPI_SEND_CMD_DEBUG			
			sprintf(str,"SPI Erase:%.2X %.2X %.2X",
						FlashPkg.addr[0],
						FlashPkg.addr[1],
						FlashPkg.addr[2]);
#endif			
		
		}
#ifdef SPI_SEND_CMD_DEBUG		
		mx25DbgMsg(str);
#endif		
	}	

}

void MX25L_SPI_1_event_handler(smp_spi_evt_type p_evt)
{
//#define	SPI_EVENT_DEBUG	
	smp_flash_package	FlashPkg;
	
	switch(p_evt){
		case SMP_SPI_EVENT_DONE:	
			if(UseDMAFlag == false){
				return;
			}
			smp_fifo_flash_pop(&flash_read_fifo, &FlashPkg);
			
			if(FlashPkg.command == MX25L_READ){
#ifdef SPI_EVENT_DEBUG				
				mx25DbgMsg("SPI READ DONE");
#endif				
				FlashPkg.flash_callback(SMP_FLASH_EVENT_READ_DONE);
			}else if(FlashPkg.command == MX25L_FAST_READ){
#ifdef SPI_EVENT_DEBUG				
				mx25DbgMsg("SPI F_READ DONE");	
#endif				
				FlashPkg.flash_callback(SMP_FLASH_EVENT_READ_DONE);
			}else if(FlashPkg.command == MX25L_PP){
#ifdef SPI_EVENT_DEBUG				
				mx25DbgMsg("SPI WRITE DONE");	
#endif				
				FlashPkg.flash_callback(SMP_FLASH_EVENT_WRITE_DONE);
			}else if(FlashPkg.command == (MX25L_SE|MX25L_BE64|MX25L_CE)){
#ifdef SPI_EVENT_DEBUG				
				mx25DbgMsg("SPI ERASE DONE");	
#endif				
				FlashPkg.flash_callback(SMP_FLASH_EVENT_ERASE_DONE);
			}

		break;
		default:
		break;
	}
}


///******************************END OF FILE*****************************/
