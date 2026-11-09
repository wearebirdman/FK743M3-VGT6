#include "w25qxx.h"
#include "quadspi.h"
#if W25QXX_ENABLE_EXAMPLE
#include "usart.h"
#endif

#include <string.h>

/* ========== 内部宏定义 ========== */
#define W25QXX_EXPECTED_MF_ID   0xEF /* 制造商 ID（Winbond） */
#define W25QXX_EXPECTED_TYPE_ID 0x40 /* 存储类型 ID（W25Qxx 系列） */
#define W25QXX_DUMMY_CYCLES     8U   /* 0x6B 固定 8 个 dummy 时钟 */
#define W25QXX_POLL_INTERVAL    0x10U /* 自动轮询间隔时钟数 */

/* ========== 超时时间配置（单位：ms） ========== */
#define W25QXX_TIMEOUT_GENERIC      100 /* 通用操作超时（指令、状态读取） */
#define W25QXX_TIMEOUT_SECTOR_ERASE 500 /* 扇区擦除超时（典型 40~400ms） */
#define W25QXX_TIMEOUT_PAGE_PROGRAM 100 /* 页编程超时（典型 0.4~3ms） */
#define W25QXX_TIMEOUT_RESET        100 /* 软复位指令超时 */
#define W25QXX_RESET_DELAY_MS       1   /* 复位后延时（ms） */
#define W25QXX_WAKE_DELAY_MS        1   /* 唤醒后延时（ms） */
#define W25QXX_BULK_BYTES_PER_MS    1024U /* 整块传输超时预算：保守吞吐下限 1MB/s */

/* ========== 内部函数声明 ========== */
static void W25Qxx_FillCmd(QSPI_CommandTypeDef *cmd, uint8_t instruction, uint32_t addr, uint32_t addr_mode,
                           uint32_t data_mode, uint32_t dummy, uint32_t nb_data);
static W25QxxErr_t W25Qxx_SendCmdOnly(uint8_t instruction, uint32_t timeout);
static uint8_t W25Qxx_CheckAddr(uint32_t addr, uint32_t len);
static W25QxxErr_t W25Qxx_ReadStatusChecked(uint8_t instruction, uint8_t *value);
static uint8_t W25Qxx_CheckProtected(void);
static uint32_t W25Qxx_TimeoutByLen(uint32_t len);
static W25QxxErr_t W25Qxx_WaitBusyTimeout(uint32_t timeout_ms);
static uint32_t W25Qxx_ReadID(void);
static W25QxxErr_t W25Qxx_Reset(void);
static W25QxxErr_t W25Qxx_EnsureQuadEnable(void);
static W25QxxErr_t W25Qxx_WriteEnable(void);
static W25QxxErr_t W25Qxx_WriteDisable(void);
static W25QxxErr_t W25Qxx_WaitBusy(void);
static W25QxxErr_t W25Qxx_PageWrite(uint32_t addr, const uint8_t *buf, uint16_t len);
static uint8_t W25Qxx_IsMemoryMapped(void);

/* ========== QSPI 底层操作 ========== */

/* 填充命令描述符的公共字段，各指令按需指定地址线与数据线宽度
 * QSPI 硬件自动生成指令/地址/dummy/数据四个阶段，无需像 SPI 那样逐字节打拍
 * cmd: 待填充的命令描述符
 * instruction: 指令码
 * addr: 24 位字节地址，无地址阶段时传 0
 * addr_mode: 地址阶段线宽，如 QSPI_ADDRESS_1_LINE、QSPI_ADDRESS_NONE
 * data_mode: 数据阶段线宽，如 QSPI_DATA_4_LINES、QSPI_DATA_NONE
 * dummy: dummy 时钟周期数
 * nb_data: 数据字节数，无数据阶段时传 0 */
static void W25Qxx_FillCmd(QSPI_CommandTypeDef *cmd, uint8_t instruction, uint32_t addr, uint32_t addr_mode,
                           uint32_t data_mode, uint32_t dummy, uint32_t nb_data)
{
    cmd->Instruction        = instruction;
    cmd->InstructionMode    = QSPI_INSTRUCTION_1_LINE;
    cmd->Address            = addr;
    cmd->AddressMode        = addr_mode;
    cmd->AddressSize        = QSPI_ADDRESS_24_BITS;
    cmd->AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
    cmd->AlternateBytes     = 0U;
    cmd->AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    cmd->DummyCycles        = dummy;
    cmd->DataMode           = data_mode;
    cmd->NbData             = nb_data;
    cmd->DdrMode            = QSPI_DDR_MODE_DISABLE;
    cmd->DdrHoldHalfCycle   = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd->SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
}

/* 发送无地址无数据的单指令（写使能/写禁止/复位/擦除/掉电等）
 * 返回: HAL 失败统一映射为超时错误 */
static W25QxxErr_t W25Qxx_SendCmdOnly(uint8_t instruction, uint32_t timeout)
{
    QSPI_CommandTypeDef cmd = {0};

    W25Qxx_FillCmd(&cmd, instruction, 0U, QSPI_ADDRESS_NONE, QSPI_DATA_NONE, 0U, 0U);

    return (HAL_QSPI_Command(&W25QXX_QSPI, &cmd, timeout) == HAL_OK) ? W25QXX_OK : W25QXX_ERR_TIMEOUT;
}

/* ========== 内部辅助函数 ========== */

/* 检查地址是否越界，0 = 越界, 1 = 合法
 * 用 len > CHIP - addr 比较，避免 addr + len 在 uint32 回绕漏检 */
static uint8_t W25Qxx_CheckAddr(uint32_t addr, uint32_t len)
{
    if (addr >= W25QXX_CHIP_SIZE)
        return 0;
    if (len > W25QXX_CHIP_SIZE - addr)
        return 0;
    return 1;
}

/* 读状态寄存器 1 或 2（带链路状态检查，供忙等待/写保护判断内部使用）
 * 返回: QSPI 失败为 W25QXX_ERR_TIMEOUT，成功时读取值写入 *value */
static W25QxxErr_t W25Qxx_ReadStatusChecked(uint8_t instruction, uint8_t *value)
{
    QSPI_CommandTypeDef cmd = {0};

    W25Qxx_FillCmd(&cmd, instruction, 0U, QSPI_ADDRESS_NONE, QSPI_DATA_1_LINE, 0U, 1U);

    if (HAL_QSPI_Command(&W25QXX_QSPI, &cmd, W25QXX_TIMEOUT_GENERIC) != HAL_OK)
        return W25QXX_ERR_TIMEOUT;
    if (HAL_QSPI_Receive(&W25QXX_QSPI, value, W25QXX_TIMEOUT_GENERIC) != HAL_OK)
        return W25QXX_ERR_TIMEOUT;

    return W25QXX_OK;
}

/* 检查芯片是否处于写保护状态（BP 位非零），0 = 未保护, 1 = 已保护
 * 读取失败时返回 0（未保护）：通信错误交由后续 WriteEnable 如实上报超时，
 * 避免把掉线芯片误判成写保护 */
static uint8_t W25Qxx_CheckProtected(void)
{
    uint8_t sr1 = 0U;

    if (W25Qxx_ReadStatusChecked(W25QXX_READ_STATUS1, &sr1) != W25QXX_OK)
        return 0;

    /* 检查 BP0~BP2 是否全为 0 */
    if (sr1 & (W25QXX_SR1_BP0 | W25QXX_SR1_BP1 | W25QXX_SR1_BP2))
        return 1;

    return 0;
}

/* 按传输长度估算超时时间：固定开销 + 保守吞吐预算 */
static uint32_t W25Qxx_TimeoutByLen(uint32_t len)
{
    return W25QXX_TIMEOUT_GENERIC + len / W25QXX_BULK_BYTES_PER_MS;
}

/* 等待芯片空闲（QSPI 硬件自动轮询状态寄存器 BUSY 位，不占用 CPU 逐字节收发）
 * 返回: W25QXX_OK = 已空闲；W25QXX_ERR_TIMEOUT = 超时仍忙 */
static W25QxxErr_t W25Qxx_WaitBusyTimeout(uint32_t timeout_ms)
{
    QSPI_CommandTypeDef     cmd = {0};
    QSPI_AutoPollingTypeDef cfg = {0};

    W25Qxx_FillCmd(&cmd, W25QXX_READ_STATUS1, 0U, QSPI_ADDRESS_NONE, QSPI_DATA_1_LINE, 0U, 1U);

    cfg.Match           = 0U;
    cfg.Mask            = W25QXX_SR1_BUSY;
    cfg.Interval        = W25QXX_POLL_INTERVAL;
    cfg.StatusBytesSize = 1U;
    cfg.MatchMode       = QSPI_MATCH_MODE_AND;
    cfg.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

    return (HAL_QSPI_AutoPolling(&W25QXX_QSPI, &cmd, &cfg, timeout_ms) == HAL_OK) ? W25QXX_OK : W25QXX_ERR_TIMEOUT;
}

/* 检查芯片是否处于内存映射模式，映射模式下写擦和间接读不可用 */
static uint8_t W25Qxx_IsMemoryMapped(void)
{
    return (W25QXX_QSPI.State == HAL_QSPI_STATE_BUSY_MEM_MAPPED) ? 1 : 0;
}

/* ========== 初始化与信息读取 ========== */

/* 软复位：清掉上一次运行可能残留的连续读或映射状态，由调用方处理返回值 */
static W25QxxErr_t W25Qxx_Reset(void)
{
    W25QxxErr_t ret = W25Qxx_SendCmdOnly(W25QXX_RESET_ENABLE, W25QXX_TIMEOUT_RESET);

    if (ret != W25QXX_OK)
        return ret;

    ret = W25Qxx_SendCmdOnly(W25QXX_RESET_DEVICE, W25QXX_TIMEOUT_RESET);
    HAL_Delay(W25QXX_RESET_DELAY_MS);

    return ret;
}

/* 置位状态寄存器 2 的四线使能位 QE
 * 原理图将 IO2/IO3 接到 WP/HOLD 脚位，QE 未置位时四线读无效，故初始化统一补齐 */
static W25QxxErr_t W25Qxx_EnsureQuadEnable(void)
{
    QSPI_CommandTypeDef cmd = {0};
    uint8_t status[2];
    uint8_t sr1 = 0U;
    uint8_t sr2 = 0U;
    W25QxxErr_t ret;

    ret = W25Qxx_ReadStatusChecked(W25QXX_READ_STATUS1, &sr1);
    if (ret != W25QXX_OK)
        return ret;

    ret = W25Qxx_ReadStatusChecked(W25QXX_READ_STATUS2, &sr2);
    if (ret != W25QXX_OK)
        return ret;

    if (sr2 & W25QXX_SR2_QE)
        return W25QXX_OK;

    /* 写状态寄存器需同时给出 SR1/SR2，SR1 原值回写以保留既有保护设置 */
    status[0] = sr1;
    status[1] = (uint8_t)(sr2 | W25QXX_SR2_QE);

    ret = W25Qxx_WriteEnable();
    if (ret != W25QXX_OK)
        return ret;

    W25Qxx_FillCmd(&cmd, W25QXX_WRITE_STATUS, 0U, QSPI_ADDRESS_NONE, QSPI_DATA_1_LINE, 0U, 2U);

    if (HAL_QSPI_Command(&W25QXX_QSPI, &cmd, W25QXX_TIMEOUT_GENERIC) != HAL_OK)
        return W25QXX_ERR_TIMEOUT;
    if (HAL_QSPI_Transmit(&W25QXX_QSPI, status, W25QXX_TIMEOUT_PAGE_PROGRAM) != HAL_OK)
        return W25QXX_ERR_TIMEOUT;

    return W25Qxx_WaitBusyTimeout(W25QXX_TIMEOUT_PAGE_PROGRAM);
}

/* 初始化 W25QXX，验证制造商 ID / 存储类型 / 容量 ID 是否匹配 */
W25QxxErr_t W25Qxx_Init(void)
{
    /* 先软复位，确保芯片不在连续读或深度掉电状态 */
    W25QxxErr_t ret = W25Qxx_Reset();
    if (ret != W25QXX_OK)
        return ret;

    uint32_t id = W25Qxx_ReadID();

    /* 检查制造商 ID 是否为 Winbond (0xEF) */
    if (((id >> 16) & 0xFF) != W25QXX_EXPECTED_MF_ID)
        return W25QXX_ERR_ID;

    /* 检查存储类型是否为 W25QXX 系列 (0x40) */
    if (((id >> 8) & 0xFF) != W25QXX_EXPECTED_TYPE_ID)
        return W25QXX_ERR_ID;

    /* 检查容量 ID 是否匹配当前定义的芯片型号（不匹配可能是型号选择错误） */
    if ((id & 0xFF) != (W25QXX_EXPECTED_ID & 0xFF))
        return W25QXX_ERR_ID;

    /* 退出掉电模式（如果进入） */
    ret = W25Qxx_WakeUp();
    if (ret != W25QXX_OK)
        return ret;

    /* 四线读依赖 QE 位 */
    ret = W25Qxx_EnsureQuadEnable();
    if (ret != W25QXX_OK)
        return ret;

    /* 清除写使能 */
    return W25Qxx_WriteDisable();
}

/* 获取芯片信息 */
void W25Qxx_GetInfo(W25QxxInfo_t *info)
{
    if (info == NULL)
        return;

    info->jedec_id = W25Qxx_ReadID();
    info->manufacturer = (uint8_t)((info->jedec_id >> 16) & 0xFF);
    info->memory_type = (uint8_t)((info->jedec_id >> 8) & 0xFF);
    info->capacity = (uint8_t)(info->jedec_id & 0xFF);

    /* 使用宏定义的容量参数（编译时计算） */
    info->size_kb = W25QXX_CHIP_SIZE / 1024;
    info->size_mb = W25QXX_CHIP_SIZE / 1024 / 1024;
    info->sector_count = W25QXX_SECTOR_COUNT;
    info->page_count = W25QXX_PAGE_COUNT;
    info->block_count = W25QXX_BLOCK_COUNT;
}

/* 读取芯片 JEDEC ID，返回 24 位 ID（制造商 | 存储类型 | 容量），QSPI 失败返回 0 */
static uint32_t W25Qxx_ReadID(void)
{
    QSPI_CommandTypeDef cmd = {0};
    uint8_t id[3] = {0};

    W25Qxx_FillCmd(&cmd, W25QXX_JEDEC_ID, 0U, QSPI_ADDRESS_NONE, QSPI_DATA_1_LINE, 0U, 3U);

    if (HAL_QSPI_Command(&W25QXX_QSPI, &cmd, W25QXX_TIMEOUT_GENERIC) != HAL_OK)
        return 0U;
    if (HAL_QSPI_Receive(&W25QXX_QSPI, id, W25QXX_TIMEOUT_GENERIC) != HAL_OK)
        return 0U;

    return ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
}

/* ========== 写使能与等待 ========== */

/* 写使能（任何写入/擦除操作前必须调用） */
static W25QxxErr_t W25Qxx_WriteEnable(void)
{
    return W25Qxx_SendCmdOnly(W25QXX_WRITE_EN, W25QXX_TIMEOUT_GENERIC);
}

/* 写禁止 */
static W25QxxErr_t W25Qxx_WriteDisable(void)
{
    return W25Qxx_SendCmdOnly(W25QXX_WRITE_DIS, W25QXX_TIMEOUT_GENERIC);
}

/* 等待芯片空闲（使用默认超时时间 W25QXX_TIMEOUT_GENERIC） */
static W25QxxErr_t W25Qxx_WaitBusy(void)
{
    return W25Qxx_WaitBusyTimeout(W25QXX_TIMEOUT_GENERIC);
}

/* ========== 数据读取 ========== */

/* 从指定地址读取数据（addr: 0 ~ W25QXX_CHIP_SIZE-1）
 * 采用 0x6B 四线快读，吞吐约为单线 SPI 的 4 倍，dummy 相位固定 8 个时钟 */
W25QxxErr_t W25Qxx_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    QSPI_CommandTypeDef cmd = {0};

    /* 参数检查 */
    if (buf == NULL || len == 0)
        return W25QXX_ERR_PARAM;

    /* 地址越界检查 */
    if (!W25Qxx_CheckAddr(addr, len))
        return W25QXX_ERR_ADDR;

    /* 映射模式下 QSPI 无法发起间接读 */
    if (W25Qxx_IsMemoryMapped())
        return W25QXX_ERR_STATE;

    W25Qxx_FillCmd(&cmd, W25QXX_QUAD_READ, addr, QSPI_ADDRESS_1_LINE, QSPI_DATA_4_LINES, W25QXX_DUMMY_CYCLES, len);

    if (HAL_QSPI_Command(&W25QXX_QSPI, &cmd, W25QXX_TIMEOUT_GENERIC) != HAL_OK)
        return W25QXX_ERR_TIMEOUT;
    if (HAL_QSPI_Receive(&W25QXX_QSPI, buf, W25Qxx_TimeoutByLen(len)) != HAL_OK)
        return W25QXX_ERR_TIMEOUT;

    return W25QXX_OK;
}

/* ========== 页写入 ========== */

/* 页写入（单次最多 256 字节）
 * 如果数据跨页，硬件会自动回绕到页首，可能导致数据覆盖，
 * 调用者应使用 W25Qxx_Write() 处理跨页 */
static W25QxxErr_t W25Qxx_PageWrite(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    QSPI_CommandTypeDef cmd = {0};

    /* 参数检查 */
    if (buf == NULL || len == 0)
        return W25QXX_ERR_PARAM;

    if (len > W25QXX_PAGE_SIZE)
        return W25QXX_ERR_PARAM;

    /* 地址越界检查 */
    if (!W25Qxx_CheckAddr(addr, len))
        return W25QXX_ERR_ADDR;

    /* 检查写保护 */
    if (W25Qxx_CheckProtected())
        return W25QXX_ERR_PROTECTED;

    W25QxxErr_t ret = W25Qxx_WriteEnable();
    if (ret != W25QXX_OK)
        return ret;

    W25Qxx_FillCmd(&cmd, W25QXX_PAGE_PROGRAM, addr, QSPI_ADDRESS_1_LINE, QSPI_DATA_1_LINE, 0U, len);

    if (HAL_QSPI_Command(&W25QXX_QSPI, &cmd, W25QXX_TIMEOUT_GENERIC) != HAL_OK)
        return W25QXX_ERR_TIMEOUT;
    /* HAL 形参非 const，驱动内唯一转换点 */
    if (HAL_QSPI_Transmit(&W25QXX_QSPI, (uint8_t *)buf, W25QXX_TIMEOUT_PAGE_PROGRAM) != HAL_OK)
        return W25QXX_ERR_TIMEOUT;

    return W25Qxx_WaitBusy();
}

/* 任意地址写入任意长度数据（自动处理跨页）
 * 自动将数据拆分为多次页写入，处理页边界。
 * 注意：写入前需要确保目标地址所在的扇区已被擦除 */
W25QxxErr_t W25Qxx_Write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    uint32_t remaining = len;
    uint32_t current_addr = addr;
    const uint8_t *current_buf = buf;

    /* 参数检查 */
    if (buf == NULL || len == 0)
        return W25QXX_ERR_PARAM;

    /* 地址越界检查 */
    if (!W25Qxx_CheckAddr(addr, len))
        return W25QXX_ERR_ADDR;

    /* 映射模式下不能写，需先调用 W25Qxx_MemoryMappedExit() */
    if (W25Qxx_IsMemoryMapped())
        return W25QXX_ERR_STATE;

    /* 检查写保护 */
    if (W25Qxx_CheckProtected())
        return W25QXX_ERR_PROTECTED;

    while (remaining > 0)
    {
        /* 计算当前页剩余空间 */
        uint16_t page_offset = current_addr & (W25QXX_PAGE_SIZE - 1);
        uint16_t page_remain = W25QXX_PAGE_SIZE - page_offset;

        /* 本次可写入的长度 */
        uint16_t chunk = (remaining > page_remain) ? page_remain : (uint16_t)remaining;

        /* 执行页写入 */
        W25QxxErr_t ret = W25Qxx_PageWrite(current_addr, current_buf, chunk);
        if (ret != W25QXX_OK)
            return ret;

        /* 更新指针和剩余长度 */
        current_addr += chunk;
        current_buf += chunk;
        remaining -= chunk;
    }

    return W25QXX_OK;
}

/* ========== 擦除操作 ========== */

/* 扇区擦除（4KB），addr 为扇区内任意地址（自动对齐到扇区首地址） */
W25QxxErr_t W25Qxx_SectorErase(uint32_t addr)
{
    QSPI_CommandTypeDef cmd = {0};

    /* 地址越界检查（先查原始地址，避免对齐回绕漏检） */
    if (addr >= W25QXX_CHIP_SIZE)
        return W25QXX_ERR_ADDR;

    /* 映射模式下不能擦除，需先调用 W25Qxx_MemoryMappedExit() */
    if (W25Qxx_IsMemoryMapped())
        return W25QXX_ERR_STATE;

    /* 自动对齐到扇区首地址 */
    uint32_t sector_addr = W25QXX_SECTOR_ALIGN(addr);

    /* 检查写保护 */
    if (W25Qxx_CheckProtected())
        return W25QXX_ERR_PROTECTED;

    W25QxxErr_t ret = W25Qxx_WriteEnable();
    if (ret != W25QXX_OK)
        return ret;

    W25Qxx_FillCmd(&cmd, W25QXX_SECTOR_ERASE, sector_addr, QSPI_ADDRESS_1_LINE, QSPI_DATA_NONE, 0U, 0U);

    if (HAL_QSPI_Command(&W25QXX_QSPI, &cmd, W25QXX_TIMEOUT_GENERIC) != HAL_OK)
        return W25QXX_ERR_TIMEOUT;

    /* 扇区擦除典型时间：40~400ms，使用 500ms 超时 */
    return W25Qxx_WaitBusyTimeout(W25QXX_TIMEOUT_SECTOR_ERASE);
}

/* ========== 电源管理 ========== */

/* 进入掉电模式（降低功耗） */
W25QxxErr_t W25Qxx_PowerDown(void)
{
    if (W25Qxx_IsMemoryMapped())
        return W25QXX_ERR_STATE;

    return W25Qxx_SendCmdOnly(W25QXX_POWER_DOWN, W25QXX_TIMEOUT_GENERIC);
}

/* 退出掉电模式（唤醒芯片），唤醒后需要等待至少 3μs 才能发送命令 */
W25QxxErr_t W25Qxx_WakeUp(void)
{
    if (W25Qxx_IsMemoryMapped())
        return W25QXX_ERR_STATE;

    W25QxxErr_t ret = W25Qxx_SendCmdOnly(W25QXX_RELEASE_POWER_DOWN, W25QXX_TIMEOUT_GENERIC);
    if (ret != W25QXX_OK)
        return ret;

    /* 等待芯片唤醒（典型 3μs，给一个安全延时 1ms） */
    HAL_Delay(W25QXX_WAKE_DELAY_MS);

    return W25QXX_OK;
}

/* ========== 内存映射（QSPI 特有） ========== */

/* 进入内存映射模式，之后读 W25QXX_MEM_BASE + addr 即可直接取数，CPU 无需参与
 * 映射区必须已由 MPU 配置为非缓存，否则读到的是缓存旧值 */
W25QxxErr_t W25Qxx_MemoryMappedEnter(void)
{
    QSPI_CommandTypeDef      cmd = {0};
    QSPI_MemoryMappedTypeDef cfg = {0};

    if (W25Qxx_IsMemoryMapped())
        return W25QXX_OK;

    W25Qxx_FillCmd(&cmd, W25QXX_QUAD_READ, 0U, QSPI_ADDRESS_1_LINE, QSPI_DATA_4_LINES, W25QXX_DUMMY_CYCLES, 0U);

    cfg.TimeOutPeriod     = 0U;
    cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;

    return (HAL_QSPI_MemoryMapped(&W25QXX_QSPI, &cmd, &cfg) == HAL_OK) ? W25QXX_OK : W25QXX_ERR_TIMEOUT;
}

/* 退出内存映射模式，写擦与间接读前必须调用 */
W25QxxErr_t W25Qxx_MemoryMappedExit(void)
{
    if (!W25Qxx_IsMemoryMapped())
        return W25QXX_OK;

    return (HAL_QSPI_Abort(&W25QXX_QSPI) == HAL_OK) ? W25QXX_OK : W25QXX_ERR_TIMEOUT;
}

/* 从映射区拷贝数据，映射区为非缓存属性，可直接 memcpy
 * addr: 芯片内字节偏移
 * buf: 接收缓冲区
 * len: 字节数 */
void W25Qxx_MemRead(uint32_t addr, void *buf, uint32_t len)
{
    if (buf == NULL || len == 0)
        return;

    if (!W25Qxx_CheckAddr(addr, len))
        return;

    memcpy(buf, (const void *)(W25QXX_MEM_BASE + addr), len);
}

/* ========== 使用示例 ========== */
#if W25QXX_ENABLE_EXAMPLE

/* W25QXX 使用示例，展示完整的读写流程：
 * 初始化芯片 -> 获取芯片信息 -> 擦除扇区 -> 写入数据 -> 读取数据 -> 验证数据 */
void W25Qxx_Example(void)
{
    /* 测试数据 */
    uint8_t write_buf[] = "Hello W25QXX!";
    uint8_t read_buf[sizeof(write_buf)];
    W25QxxInfo_t info;
    W25QxxErr_t ret;

    /* 1. 初始化 */
    ret = W25Qxx_Init();
    if (ret != W25QXX_OK)
    {
        Uart_Printf("W25QXX Init Failed! Error: %d\r\n", ret);
        return;
    }

    /* 2. 获取芯片信息 */
    W25Qxx_GetInfo(&info);
    Uart_Printf("========== W25QXX Info ==========\r\n");
    Uart_Printf("JEDEC ID:     0x%06lX\r\n", info.jedec_id);
    Uart_Printf("Manufacturer: 0x%02X\r\n", info.manufacturer);
    Uart_Printf("Memory Type:  0x%02X\r\n", info.memory_type);
    Uart_Printf("Capacity ID:  0x%02X\r\n", info.capacity);
    Uart_Printf("Capacity:     %lu MB (%lu bytes)\r\n", info.size_mb, (uint32_t)W25QXX_CHIP_SIZE);
    Uart_Printf("Sector Size:  %u KB\r\n", W25QXX_SECTOR_SIZE / 1024);
    Uart_Printf("Sector Count: %lu\r\n", info.sector_count);
    Uart_Printf("Block Size:   %u KB\r\n", W25QXX_BLOCK_SIZE / 1024);
    Uart_Printf("Block Count:  %lu\r\n", info.block_count);
    Uart_Printf("Page Size:    %u B\r\n", W25QXX_PAGE_SIZE);
    Uart_Printf("Page Count:   %lu\r\n", info.page_count);
    Uart_Printf("==================================\r\n\r\n");

    /* 3. 擦除扇区（使用第一个扇区） */
    Uart_Printf("Erasing sector at address 0x%06X...\r\n", W25QXX_TEST_ADDR);
    ret = W25Qxx_SectorErase(W25QXX_TEST_ADDR);
    if (ret != W25QXX_OK)
    {
        Uart_Printf("Erase Failed! Error: %d\r\n", ret);
        return;
    }
    Uart_Printf("Sector erased OK!\r\n\r\n");

    /* 4. 写入数据（使用跨页写入函数） */
    Uart_Printf("Writing %u bytes to address 0x%06X...\r\n", (uint32_t)sizeof(write_buf), W25QXX_TEST_ADDR);
    ret = W25Qxx_Write(W25QXX_TEST_ADDR, write_buf, sizeof(write_buf));
    if (ret != W25QXX_OK)
    {
        Uart_Printf("Write Failed! Error: %d\r\n", ret);
        return;
    }
    Uart_Printf("Write OK!\r\n\r\n");

    /* 5. 读取数据 */
    memset(read_buf, 0, sizeof(read_buf));
    Uart_Printf("Reading %u bytes from address 0x%06X...\r\n", (uint32_t)sizeof(read_buf), W25QXX_TEST_ADDR);
    ret = W25Qxx_Read(W25QXX_TEST_ADDR, read_buf, sizeof(read_buf));
    if (ret != W25QXX_OK)
    {
        Uart_Printf("Read Failed! Error: %d\r\n", ret);
        return;
    }
    Uart_Printf("Read OK!\r\n\r\n");

    /* 6. 验证数据 */
    if (memcmp(write_buf, read_buf, sizeof(write_buf)) == 0)
    {
        Uart_Printf("Data verified OK!\r\n");
        Uart_Printf("Written: \"%s\"\r\n", write_buf);
        Uart_Printf("Read:    \"%s\"\r\n", read_buf);
    }
    else
    {
        Uart_Printf("Data verification FAILED!\r\n");
        Uart_Printf("Written: \"%s\"\r\n", write_buf);
        Uart_Printf("Read:    \"%s\"\r\n", read_buf);
    }
}

#endif /* W25QXX_ENABLE_EXAMPLE */
