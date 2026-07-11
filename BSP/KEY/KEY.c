#include "KEY.h"

/* 已注册的按键配置表，由 Enroll 层通过 KEY_Register 提供。 */
static const KEY_Config_t *s_keyConfigTable;
/* 当前配置表中的按键数量。 */
static uint8_t s_keyConfigCount;

/* Key_Num：按键扫描完成后暂存的一次有效键值。
 * 仅在按键事件稳定后写入，读取后会自动清空。
 *
 * 注意：
 *   Key_Num 在 Key_Tick() 中写入，而 Key_GetNum()/KEY_GetEvent()
 *   通常在主循环中读取，所以这里加 volatile，避免编译器优化导致主循环读不到最新值。
 */
static volatile uint8_t Key_Num;

/* Key：对外暴露的当前最新按键事件。
 *
 * 说明：
 *   0 = 本次没有新的按键事件
 *   1 = KEY1 释放后产生的一次事件
 *   2 = KEY2 释放后产生的一次事件
 *   3 = KEY3 释放后产生的一次事件
 *   4 = KEY4 释放后产生的一次事件
 *
 * 注意：
 *   这是“一次性事件”，不是累计计数。
 *   每次主循环调用 key_Get() 后，如果没有新事件，Key 会被置 0。
 */
volatile uint8_t Key = 0U;

/* 按键扫描内部状态：用于消抖和状态变化检测。 */
#define KEY_DEBOUNCE_COUNT 3U		   /* 消抖计数 */
static uint8_t s_keyLastRawState;      /* 最近一次采样的原始按键状态。 */
static uint8_t s_keyStableState;       /* 当前稳定的按键状态。 */
static uint8_t s_keyPrevStableState;   /* 上一次稳定的按键状态。 */
static uint8_t s_keyDebounceCount;     /* 当前原始状态连续稳定计数。 */
static uint8_t s_keyTickCount;         /* 用于限制 Key_Tick 的扫描频率。 */

/*
 * 从注册表里取指定序号的按键配置。
 * 参数 index 由 0 开始，对应第 1 个、第 2 个按键。
 */
static const KEY_Config_t *KEY_GetConfig(uint8_t index)
{
	if ((s_keyConfigTable == 0) || (index >= s_keyConfigCount))
	{
		return 0;
	}

	return &s_keyConfigTable[index];
}

/*
 * Key_GetNum：读取一次按键事件结果。
 * 仅当 Key_Tick 检测到稳定的按键释放事件时才会写入 Key_Num。
 * 读取后会清空 Key_Num，避免重复上报。
 */
static uint8_t Key_GetNum(void)
{
	uint8_t Temp;

	if (Key_Num != 0U)
	{
		Temp = (uint8_t)Key_Num;
		Key_Num = 0U;
		return Temp;
	}

	return 0U;
}

/*
 * Key_GetState：读取当前按键电平状态。
 * 对于低电平按下的按键，gpioRead 返回 0 表示按下。
 * 返回值：0 = 未按下；1 = KEY1；2 = KEY2；3 = KEY3；4 = KEY4。
 */
static uint8_t Key_GetState(void)
{
	uint8_t i;
	const KEY_Config_t *config;

	for (i = 0U; i < s_keyConfigCount; ++i)
	{
		config = KEY_GetConfig(i);
		if ((config != 0) && (config->gpioRead != 0))
		{
			/* 0 表示按键按下，1 表示松开（低电平按下模式）。 */
			if (config->gpioRead(config->port, config->pin) == 0U)
			{
				return (uint8_t)(config->id + 1U);
			}
		}
	}

	return 0U;
}

/*
 * KEY_Register：把 Enroll 层提供的按键资源表保存下来。
 * 这里只登记映射，不直接碰硬件。
 */
void KEY_Register(const KEY_Config_t *configTable, uint8_t count)
{
	s_keyConfigTable = 0;
	s_keyConfigCount = 0U;

	if ((configTable == 0) || (count == 0U))
	{
		return;
	}

	s_keyConfigTable = configTable;
	s_keyConfigCount = count;
}

/*
 * KEY_Init：初始化已注册的按键 GPIO。
 * 该函数会调用每个按键配置项中的 gpioInit，把按键引脚设置成可读取状态。
 * 同时清除上电后的按键事件缓存，避免初始化完成后误报 KEY1。
 */
void KEY_Init(void)
{
	uint8_t i;
	const KEY_Config_t *config;

	for (i = 0U; i < s_keyConfigCount; ++i)
	{
		config = KEY_GetConfig(i);
		if ((config != 0) && (config->gpioInit != 0))
		{
			config->gpioInit(config->port, config->pin);
		}
	}

	/* 清理初始化时的按键状态，避免上电后误判 */
	Key_Num = 0U;
	Key = 0U;
	s_keyLastRawState = Key_GetState();
	s_keyStableState = s_keyLastRawState;
	s_keyPrevStableState = s_keyLastRawState;
	s_keyDebounceCount = KEY_DEBOUNCE_COUNT;
	s_keyTickCount = 0U;
}

/*
 * Key_Tick：按键扫描函数。
 *
 * 调用位置：
 *   放在 1ms 定时器回调里周期调用。
 *
 * 内部逻辑：
 *   1. 每 20 次 tick 采样一次，也就是约 20ms 采样一次；
 *   2. 通过 KEY_DEBOUNCE_COUNT 做连续稳定检测；
 *   3. 只有按键从“按下”稳定变为“释放”时，才产生一次按键事件；
 *   4. 长按不重复上报，松开时只上报一次。
 */
void Key_Tick(void)
{
	uint8_t rawState;

	s_keyTickCount++;
	if (s_keyTickCount < 20U)
	{
		return;
	}
	s_keyTickCount = 0U;

	rawState = Key_GetState();
	if (rawState == s_keyLastRawState)
	{
		/* 原始读数连续稳定，则增加消抖计数。 */
		if (s_keyDebounceCount < KEY_DEBOUNCE_COUNT)
		{
			s_keyDebounceCount++;
		}
	}
	else
	{
		/* 原始读数变化，重新开始消抖计数。 */
		s_keyLastRawState = rawState;
		s_keyDebounceCount = 1U;
	}

	if ((s_keyDebounceCount >= KEY_DEBOUNCE_COUNT) &&
	    (s_keyLastRawState != s_keyStableState))
	{
		s_keyPrevStableState = s_keyStableState;
		s_keyStableState = s_keyLastRawState;

		/* 只有按键释放时才记为一次有效按键事件，避免长按重复上报。 */
		if ((s_keyStableState == 0U) && (s_keyPrevStableState != 0U))
		{
			Key_Num = s_keyPrevStableState;
		}
	}
}

/*
 * KEY_GetEvent：读取一次按键事件。
 *
 * 返回值：
 *   0 = 没有新的按键事件
 *   1 = KEY1
 *   2 = KEY2
 *   3 = KEY3
 *   4 = KEY4
 *
 * 读取后会自动清空事件。
 * 后面做任务状态机时，建议优先使用这个函数，而不是直接判断全局 Key。
 */
uint8_t KEY_GetEvent(void)
{
	return Key_GetNum();
}

/*
 * key_Get：把扫描得到的按键事件同步到全局 Key。
 *
 * 重要变化：
 *   旧版是 Key = Key + 1；
 *   现在改为一次性事件：
 *
 *      有新按键事件：Key = 1/2/3/4
 *      没有新事件  ：Key = 0
 *
 * 这样后面做 FIND / TASK2 / TASK3 状态机时不会被累计计数干扰。
 */
void key_Get(void)
{
	Key = KEY_GetEvent();
}
