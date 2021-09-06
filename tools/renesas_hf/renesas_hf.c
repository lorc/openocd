// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2015-2021, Renesas Electronics Corporation
 * Copyright (c) 2021, EPAM Systems
 *
 * This tool is based on Renesas OP-TEE HyperFlash driver
 */

#include <stdint.h>

#define CMD_READ_ID     0
#define CMD_ERASE       1
#define CMD_READ_DATA   2
#define CMD_WRITE_DATA  3

#define RET_OK          0
#define RET_ERR         1

#define BIT(x)          ( 1U << x)

/* Common control register */
#define RPC_CMNCR	0x0000U
/* Data read control register */
#define RPC_DRCR	0x000CU
/* Data read command setting register */
#define RPC_DRCMR	0x0010U
/* Data read enable setting register */
#define RPC_DREAR	0x0014U
#define	RPC_DROPR	0x0018U
/* Data read enable setting register */
#define RPC_DRENR	0x001CU
/* Manual mode control register */
#define RPC_SMCR	0x0020U
/* Manual mode command setting register */
#define RPC_SMCMR	0x0024U
/* Manual mode address setting register */
#define RPC_SMADR	0x0028U
/* Manual mode option setting register */
#define RPC_SMOPR	0x002CU
/* Manual mode enable setting register */
#define RPC_SMENR	0x0030U
/* Manual mode read data register 0 */
#define RPC_SMRDR0	0x0038U
/* Manual mode read data register 1 */
#define RPC_SMRDR1	0x003CU
/* Manual mode write data register 0 */
#define RPC_SMWDR0	0x0040U
/* Common status register */
#define RPC_CMNSR	0x0048U
/* Data read dummy cycle setting register */
#define RPC_DRDMCR	0x0058U
/* Data read DDR enable register */
#define RPC_DRDRENR	0x005CU
/* Manual mode dummy cycle setting register */
#define RPC_SMDMCR	0x0060U
/* Manual mode DDR enable register */
#define RPC_SMDRENR	0x0064U
/* PHY control register */
#define RPC_PHYCNT	0x007CU
/* Offset */
#define RPC_OFFSET1	0x0080U
/* PHY interrupt register */
#define RPC_PHYINT	0x0088U
/* Write Buffer output base address */
#define RPC_WB_OUT_BASE 0x8000U
/* HyperFlash write command control */

#define CPG_CPGWPR      0xE6150900U
#define CPG_RPCCKCR     0xE6150238U
#define CPG_SRCR9       0XE6150924U
#define CPG_SRSTCLR9	0xE6150964U

/* CPG Write Protect Register */
#define CPG_CPGWPR_WPRTCT_MASK	(0xFFFFFFFFU)
/* RPC-IF Clock Frequency Control Register */
/* RPC-IF clock (RPC, RPCD2) Frequency Division Ratio */
#define CPG_RPCCKCR_DIV_MASK	(0x0000001FU)
/* RPC Clock Stop */
#define CPG_RPCCKCR_CKSTP_MASK	(0x00000100U)
/* RPCD2 Clock Stop */
#define CPG_RPCCKCR_CKSTP2_MASK	(0x00000200U)
/* RPC-IF Clock Frequency Control Register Mask bit */
#define CPG_RPCCKCR_MASK_BIT	(CPG_RPCCKCR_DIV_MASK | CPG_RPCCKCR_CKSTP_MASK \
                                 | CPG_RPCCKCR_CKSTP2_MASK)

/* read byte count, offset byte count */
#define FLASH_DATA_READ_BYTE_COUNT_2	2U
#define FLASH_DATA_READ_BYTE_COUNT_4	4U
#define FLASH_DATA_READ_BYTE_COUNT_8	8U
#define FLASH_DATA_OFFSET_BYTE_8	    8U

/* bit shift count */
#define FLASH_DATA_BIT_SHIFT_8		8U
#define EXT_ADDR_BIT_SHIFT_9		9U

#define FL_DEVICE_BUSY		0
#define FL_DEVICE_READY		1
#define FL_DEVICE_ERR		2

/* access size */
#define WORD_SIZE		0x00000004U
#define WRITE_BUFF_SIZE		0x00000100U
#define ERASE_SIZE_256KB	0x00040000U
#define EXT_ADD_BORDER_SIZE_64MB 0x04000000U

#define HYPER_FL_UNLOCK1_ADD		0x555U
#define HYPER_FL_UNLOCK1_DATA		((uint32_t)0xAAU << (uint32_t)24U)
#define HYPER_FL_UNLOCK2_ADD		0x2AAU
#define HYPER_FL_UNLOCK2_DATA		((uint32_t)0x55U << (uint32_t)24U)
#define HYPER_FL_UNLOCK3_ADD		0x555U
#define HYPER_FL_RESET_COM		((uint32_t)0xF0U << (uint32_t)24U)
#define	HYPER_FL_WORD_PROGRAM_COM	((uint32_t)0xA0U << (uint32_t)24U)
#define HYPER_FL_ID_ENTRY_COM		((uint32_t)0x90U << (uint32_t)24U)
#define	HYPER_FL_RD_STATUS_COM		((uint32_t)0x70U << (uint32_t)24U)
#define HYPER_FL_ERASE_1ST_COM		((uint32_t)0x80U << (uint32_t)24U)
#define HYPER_FL_SECTOR_ERASE_COM	((uint32_t)0x30U << (uint32_t)24U)

/* SMADR Register address */
#define HYPER_FL_SMADR_TOP_ADD		0x00000000U

/* Base address for RPChf */
static uint32_t base;
static uint32_t phycnt_reg;

static void io_write32_direct(uint64_t addr, uint32_t val)
{
        *((volatile uint32_t*)addr) = val;
}

static uint32_t io_read32_direct(uint64_t addr)
{
        return *((volatile uint32_t*)addr);
}

static void io_write32(uint32_t offs, uint32_t val)
{
        io_write32_direct(base + offs, val);
}

static uint32_t io_read32(uint32_t offs)
{
        return io_read32_direct(base + offs);
}

static void wait_for_tend(void)
{
        uint32_t reg;

        /*
         * We don't need to bother with timeouts. Debugger will halt
         * us in case of any problem.
         */
        while(1) {
                reg = io_read32(RPC_CMNSR);
                if ((reg & BIT(0)))
                        return;
        }
}

/* TODO: Only H3/M3/M3N are supported now. No support for D3/E3 */
static void set_rpc_clock_mode(void)
{
        uint32_t dataL = 0x00000017U;	/* RPC clock 40MHz */;
        uint32_t reg;

        io_write32_direct(CPG_CPGWPR,
                          (io_read32_direct(CPG_CPGWPR) &
                           (~CPG_CPGWPR_WPRTCT_MASK)) | (~dataL));
        io_write32_direct(CPG_RPCCKCR,
                          (io_read32_direct(CPG_RPCCKCR) &
                           (~CPG_RPCCKCR_MASK_BIT)) | dataL);

        while(1) {
                reg = io_read32_direct(CPG_RPCCKCR);
                if ((reg & CPG_RPCCKCR_MASK_BIT) == dataL)
                        break;
        }
}

static void hyper_flash_set_command(uint32_t manual_set_addr,
                                    uint32_t command)
{
        io_write32(RPC_PHYCNT, (0x80030263U | phycnt_reg));
        /*
         * bit31  CAL         =  1 : PHY calibration
         * bit1-0 PHYMEM[1:0] = 11 : HyperFlash
         */

        io_write32(RPC_CMNCR, 0x81FF7301U);
        /*
         * bit31  MD       =  1 : Manual mode
         * bit1-0 BSZ[1:0] = 01 : QSPI Flash x 2 or HyperFlash
         */

        io_write32(RPC_SMCMR, 0x00000000U);
        /*
         * bit23-21 CMD[7:5] = 000 : CA47-45 = 000 =>
         *                                      Write/memory space/WrrapedBrst
         */

        io_write32(RPC_SMADR, manual_set_addr);
        io_write32(RPC_SMOPR, 0x00000000U);
        /*
         * CA15-3(Reserved) = all 0
         */

        io_write32(RPC_SMDRENR, 0x00005101U);
        /*
         * bit14-12 HYPE =101:Hyperflash mode
         * bit8 ADDRE  = 1 : Address DDR transfer
         * bit0 SPIDRE = 1 : DATA DDR transfer
         */

        io_write32(RPC_SMENR, 0xA2225408U);
        /*
         * bit31-30 CDB[1:0]   =   10 : 4bit width command
         * bit25-24 ADB[1:0]   =   10 : 4bit width address
         * bit17-16 SPIDB[1:0] =   10 : 4bit width transfer data
         * bit15    DME        =    0 : dummy cycle disable
         * bit14    CDE        =    1 : Command enable
         * bit12    OCDE       =    1 : Option Command enable
         * bit11-8  ADE[3:0]   = 0100 : ADR[23:0] output (24 Bit Address)
         * bit7-4   OPDE[3:0]  = 0000 : Option data disable
         * bit3-0   SPIDE[3:0] = 1000 : 16bit transfer
         */

        io_write32(RPC_SMWDR0, command);

        io_write32(RPC_SMCR, 0x00000003U);
        /*
         * bit2     SPIRE      = 0 : Data read disable
         * bit1     SPIWE      = 1 : Data write enable
         * bit0     SPIE       = 1 : SPI transfer start
         */

        wait_for_tend();
}

static void hyper_flash_set_disable_write_protect(void)
{
        uint32_t dataL;

        dataL = io_read32(RPC_PHYINT);

        /*
         * bit1:WPVAL(0:RPC_WP#=H(Protect Disable),1:RPC_WP#=L(Protect Enable))
         */
        if ((dataL & BIT(1)) != 0U) {
                dataL &= ~BIT(1);
                io_write32(RPC_PHYINT, dataL);
        }
}

static void hyper_flash_reset_to_read_mode(void)
{
        /* Reset / ASO Exit */
        hyper_flash_set_command(HYPER_FL_SMADR_TOP_ADD,
                                HYPER_FL_RESET_COM);
}

static void hyper_flash_read_register_data(uint32_t manual_set_addr,
                                           uint32_t *read_data,
                                           uint32_t byte_count)
{
        io_write32(RPC_PHYCNT, (0x80030263U | phycnt_reg));
        /*
         * bit31  CAL         =  1 : PHY calibration
         * bit1-0 PHYMEM[1:0] = 11 : HyperFlash
         */

        io_write32(RPC_CMNCR, 0x81FF7301U);
        /*
         * bit31  MD       =  1 : Manual mode
         * bit1-0 BSZ[1:0] = 01 : QSPI Flash x 2 or HyperFlash
         */

        io_write32(RPC_SMCMR, 0x00800000U);
        /*
         * bit23-21 CMD[7:5] = 100 : CA47-45 = 100 =>
         *                                      Read/memory space/WrrapedBrst
         */

        io_write32(RPC_SMADR, (manual_set_addr>>1U));
        /*
         * ByteAddress(8bit) => WordAddress(16bit)
         */

        io_write32(RPC_SMOPR, 0x00000000U);
        /*
         * CA15-3(Reserved) = all 0
         */

        io_write32(RPC_SMDMCR, 0x0000000EU);
        /*
         *                           15 cycle dummy wait
         */

        io_write32(RPC_SMDRENR, 0x00005101U);
        /*
         * bit8 ADDRE  = 1 : Address DDR transfer
         * bit0 SPIDRE = 1 : DATA DDR transfer
         */

        switch (byte_count) {
                /* 2byte Read */
        case FLASH_DATA_READ_BYTE_COUNT_2:
                io_write32(RPC_SMENR, 0xA222D408U);
                /* bit3-0   SPIDE[3:0] = 1000 : 16bit transfer*/
                break;
                /* 4byte Read */
        case FLASH_DATA_READ_BYTE_COUNT_4:
                io_write32(RPC_SMENR, 0xA222D40CU);
                /* bit3-0   SPIDE[3:0] = 1100 : 32bit transfer */
                break;
                /* 8byte Read */
        case FLASH_DATA_READ_BYTE_COUNT_8:
                io_write32(RPC_SMENR, 0xA222D40FU);
                /*
                 * bit31-30 CDB[1:0]   =   10 : 4bit width command
                 * bit25-24 ADB[1:0]   =   10 : 4bit width address
                 * bit17-16 SPIDB[1:0] =   10 : 4bit width transfer data
                 * bit15    DME        =    1 : dummy cycle enable
                 * bit14    CDE        =    1 : Command enable
                 * bit12    OCDE       =    1 : Option Command enable
                 * bit11-8  ADE[3:0]   = 0100 : ADR[23:0]output(24Bit Address)
                 * bit7-4   OPDE[3:0]  = 0000 : Option data disable
                 * bit3-0   SPIDE[3:0] = 1111 : 64bit transfer
                 */
                break;
        default:
                break;
        }

        io_write32(RPC_SMCR, 0x00000005U);
        /*
         * bit2     SPIRE      = 1 : Data read enable
         * bit1     SPIWE      = 0 : Data write disable
         * bit0     SPIE       = 1 : SPI transfer start
         */

        wait_for_tend();

        if (byte_count == FLASH_DATA_READ_BYTE_COUNT_8) {
                read_data[1] = io_read32(RPC_SMRDR0);
                /* read data[63:32] */
        }

        read_data[0] = io_read32(RPC_SMRDR1);
        /* read data[31:0] */
}

static uint32_t check_if_fl_op_ended(void)
{
        uint32_t read_data[2];
        uint32_t read_status;
        uint32_t ret;

        /* 1st command write */
        hyper_flash_set_command(HYPER_FL_UNLOCK1_ADD,
                                HYPER_FL_RD_STATUS_COM);

        hyper_flash_read_register_data(HYPER_FL_SMADR_TOP_ADD,
                                       read_data, FLASH_DATA_READ_BYTE_COUNT_8);
        read_status =
                (((read_data[0] & 0xFF000000U)>>FLASH_DATA_BIT_SHIFT_8))
                | ((read_data[0] & 0x00FF0000U)<<FLASH_DATA_BIT_SHIFT_8)
                | ((read_data[0] & 0x0000FF00U)>>FLASH_DATA_BIT_SHIFT_8)
                | ((read_data[0] & 0x000000FFU)<<FLASH_DATA_BIT_SHIFT_8);

        read_status = read_status & 0x0000FFFFU;

        if ((read_status & BIT(7)) != 0U) {
                ret = FL_DEVICE_READY;
        } else {
                ret = FL_DEVICE_BUSY;
        }

        return ret;
}

static void wait_for_fl_op_end(void)
{
        /*
         * We don't need to bother with timeouts. Debugger will halt
         * us in case of any problem.
         */
        while(check_if_fl_op_ended() != FL_DEVICE_READY);
}

static void hyper_flash_write_buffer(uint32_t manual_set_addr,
                                     uint32_t *data)
{
        uintptr_t offset;

        io_write32(RPC_DRCR, 0x011F0301U);
        /*
         * bit9   RCF         =  1 : Read Cache Clear
         */

        io_write32(RPC_PHYCNT, (0x80030277U | phycnt_reg));
        /*
         * bit31  CAL         =  1 : PHY calibration
         * bit2   WBUF        =  1 : Write Buffer Enable
         * bit1-0 PHYMEM[1:0] = 11 : HyperFlash
         */

        for (offset = 0U; offset < WRITE_BUFF_SIZE;
             offset += WORD_SIZE) {
                io_write32((RPC_WB_OUT_BASE + offset), *data);
                data++;
        }

        io_write32(RPC_CMNCR, 0x81FF7301U);
        /*
         * bit31  MD       =  1 : Manual mode
         * bit1-0 BSZ[1:0] = 01 : QSPI Flash x 2 or HyperFlash
         */

        io_write32(RPC_SMCMR, 0x00000000U);
        /*
         * bit23-21 CMD[7:5] = 000 : CA47-45 = 000 =>
         *                                      Write/memory space/WrrapedBrst
         */

        io_write32(RPC_SMADR, manual_set_addr);
        io_write32(RPC_SMOPR, 0x00000000U);
        /*
         * CA15-3(Reserved) = all 0
         */

        io_write32(RPC_SMDRENR, 0x00005101U);
        /*
         * bit8 ADDRE  = 1 : Address DDR transfer
         * bit0 SPIDRE = 1 : DATA DDR transfer
         */

        io_write32(RPC_SMENR, 0xA222540FU);
        /*
         * bit31-30 CDB[1:0]   =   10 : 4bit width command
         * bit25-24 ADB[1:0]   =   10 : 4bit width address
         * bit17-16 SPIDB[1:0] =   10 : 4bit width transfer data
         * bit15    DME        =    0 : dummy cycle disable
         * bit14    CDE        =    1 : Command enable
         * bit12    OCDE       =    1 : Option Command enable
         * bit11-8  ADE[3:0]   = 0100 : ADR[23:0] output (24 Bit Address)
         * bit7-4   OPDE[3:0]  = 0000 : Option data disable
         * bit3-0   SPIDE[3:0] = 1111 : 64bit transfer
         */

        io_write32(RPC_SMCR, 0x00000003U);
        /*
         * bit2     SPIRE      = 0 : Data read disable
         * bit1     SPIWE      = 1 : Data write enable
         * bit0     SPIE       = 1 : SPI transfer start
         */

        wait_for_tend();

        io_write32(RPC_PHYCNT, 0x00030273U);
        /*
         * bit31  CAL         =  0 : No PHY calibration
         * bit2   WBUF        =  0 : Write Buffer Disable
         * bit1-0 PHYMEM[1:0] = 11 : HyperFlash
         */

        io_write32(RPC_DRCR, 0x011F0301U);
        /*
         * bit9   RCF         =  1 : Read Cache Clear
         */
}

static void hyper_flash_request_write_buffer(uint32_t flash_addr,
                                             uint32_t* data)
{
        hyper_flash_set_command(HYPER_FL_UNLOCK1_ADD,
                                HYPER_FL_UNLOCK1_DATA);

        /* 2nd command write */
        hyper_flash_set_command(HYPER_FL_UNLOCK2_ADD,
                                HYPER_FL_UNLOCK2_DATA);

        /* 3rd command write */
        hyper_flash_set_command(HYPER_FL_UNLOCK3_ADD,
                                HYPER_FL_WORD_PROGRAM_COM);


        /* 4th command write */
        flash_addr = (flash_addr/2U);

        hyper_flash_write_buffer(flash_addr, data);

        wait_for_fl_op_end();
}

static void hyper_flash_erase_sector(uint32_t sector_addr)
{
        /* 1st command write */
        hyper_flash_set_command(HYPER_FL_UNLOCK1_ADD,
                                HYPER_FL_UNLOCK1_DATA);

        /* 2nd command write */
        hyper_flash_set_command(HYPER_FL_UNLOCK2_ADD,
                                HYPER_FL_UNLOCK2_DATA);

        /* 3rd command write */
        hyper_flash_set_command(HYPER_FL_UNLOCK3_ADD,
                                HYPER_FL_ERASE_1ST_COM);
        /* 4th command write */
        hyper_flash_set_command(HYPER_FL_UNLOCK1_ADD,
                                HYPER_FL_UNLOCK1_DATA);
        /* 5th command write */
        hyper_flash_set_command(HYPER_FL_UNLOCK2_ADD,
                                HYPER_FL_UNLOCK2_DATA);

        /* 6th Command command write */
        hyper_flash_set_command((sector_addr>>1U),
                                HYPER_FL_SECTOR_ERASE_COM);

        /* BIT7: Device Ready Bit (0=Busy, 1=Ready) */
        wait_for_fl_op_end();
}

static void hyper_flash_read_device_id(uint32_t *read_device_id)
{
        uint32_t read_data[2];
        uint32_t set_addr;

        /* 1st command write */
        hyper_flash_set_command(HYPER_FL_UNLOCK1_ADD,
                                HYPER_FL_UNLOCK1_DATA);

        /* 2nd command write */
        hyper_flash_set_command(HYPER_FL_UNLOCK2_ADD,
                                HYPER_FL_UNLOCK2_DATA);
        /* 3rd command write */
        hyper_flash_set_command(HYPER_FL_UNLOCK3_ADD,
                                HYPER_FL_ID_ENTRY_COM);

        for (set_addr = 0U; set_addr < (FLASH_DATA_OFFSET_BYTE_8*2U);
             set_addr += FLASH_DATA_OFFSET_BYTE_8) {

                hyper_flash_read_register_data(set_addr,
                                               read_data,
                                               FLASH_DATA_READ_BYTE_COUNT_8);

                if (set_addr == 0U) {
                        *read_device_id =
                                (((read_data[0]&0xFF000000U)>>FLASH_DATA_BIT_SHIFT_8) |
                                 ((read_data[0]&0x00FF0000U)<<FLASH_DATA_BIT_SHIFT_8) |
                                 ((read_data[0]&0x0000FF00U)>>FLASH_DATA_BIT_SHIFT_8) |
                                 ((read_data[0]&0x000000FFU)<<FLASH_DATA_BIT_SHIFT_8));
                }
        }

        hyper_flash_reset_to_read_mode();
}

__attribute__((noreturn)) static void stop(uint32_t ret1, uint32_t ret2, uint32_t ret3,
                                           uint32_t ret4)
{
        register uint64_t x0 asm ("x0") = ret1;
        register uint64_t x1 asm ("x1") = ret2;
        register uint64_t x2 asm ("x2") = ret3;
        register uint64_t x3 asm ("x3") = ret4;
        asm volatile("hlt #0"::"r" (x0), "r" (x1), "r" (x2), "r" (x3));
        while(1);
}

static void erase(uint32_t addr, uint32_t len)
{
        if (len % ERASE_SIZE_256KB)
                len += ERASE_SIZE_256KB - len % ERASE_SIZE_256KB;

        while (len > 0)
        {
                hyper_flash_erase_sector(addr);
                len -= ERASE_SIZE_256KB;
                addr += ERASE_SIZE_256KB;
        }
        stop(RET_OK, 0, 0, 0);
}

static void read_data(uint32_t addr, uint32_t len, uint8_t* data)
{
        stop(RET_ERR, 0, 0, 0);
}

static void write_data(uint32_t addr, uint32_t len, uint8_t* data)
{
        hyper_flash_set_disable_write_protect();

        /* RPC Write Buffer size : 256byte , and rest size writing */
        while (len > 0) {
                /* last chunk */
                /* HACK: We assume that remainder is present and filed with FFs */
                if (len < WRITE_BUFF_SIZE) {
                        len = WRITE_BUFF_SIZE;
                }
                /* HACK: We assume that 'data' is 32-bit aligned  */
                hyper_flash_request_write_buffer(addr, (uint32_t*)data);
                addr += WRITE_BUFF_SIZE;
                data += WRITE_BUFF_SIZE;
                len -= WRITE_BUFF_SIZE;
        }

        stop(RET_OK, 0, 0, 0);
}

static void read_id(uint32_t base_addr)
{
        uint32_t device_id;

        hyper_flash_read_device_id(&device_id);

        stop(RET_OK, device_id, 0, 0);
}

__attribute__ ((section (".entry"))) void _entry(uint32_t base_addr, uint32_t cmd, uint32_t addr,
                                                 uint32_t len, uint8_t *data)
{
        base = base_addr;
        set_rpc_clock_mode();

        switch(cmd)
        {
        case CMD_READ_ID:
                read_id(base_addr);
                break;
        case CMD_ERASE:
                erase(addr, len);
                break;
        case CMD_READ_DATA:
                read_data(addr, len, data);
                break;
        case CMD_WRITE_DATA:
                write_data(addr, len, data);
                break;
        }

        stop(RET_ERR, 0, 0, 0);
}
