//
// Created by YZH on 2026/9/27.
//

#include "remote_input.h"
#include <stddef.h>

/**
 * @brief 检查单个摇杆轴的归一化数值是否有效。
 * @param value 待检查的轴值，允许范围为 -1.0f 到 1.0f。
 * @return 在允许范围内返回 true；超出范围或为 NaN 时返回 false。
 */
static bool AxisValid(float value)
{
    // 两个边界条件都成立才有效；NaN 的比较结果为 false，也会被拒绝。
    return value >= -1.0f && value <= 1.0f;
}

/**
 * @brief 初始化遥控输入管理对象，清空两路状态并默认不选择遥控来源。
 * @param input 遥控输入管理对象指针；为 NULL 时不执行操作。
 */
void RemoteInput_Init(RemoteInput_t *input)
{
    // 不访问空指针，避免初始化时发生非法内存访问。
    if (input == NULL)
    {
        return;
    }

    // 清空手机、手柄的状态；selected 也随之变为 REMOTE_SOURCE_NONE。
    *input = (RemoteInput_t){0};
}

/**
 * @brief 选择当前使用的遥控来源。
 * @param input 遥控输入管理对象指针；为 NULL 时不执行操作。
 * @param source 要选择的来源；非法值按无来源处理。
 * @note 重复选择当前来源不会清空状态；切换来源时清空新来源的旧数据，
 *       必须等该来源收到新的一帧有效数据后才能使用其命令。
 */
void RemoteInput_Select(RemoteInput_t *input, RemoteSource_t source)
{
    // 没有管理对象时无法切换来源。
    if (input == NULL)
    {
        return;
    }

    // 只接受定义过的来源；非法值统一改为“未选择”。
    if (source != REMOTE_SOURCE_NONE &&
        source != REMOTE_SOURCE_HANDHELD &&
        source != REMOTE_SOURCE_PHONE)
    {
        source = REMOTE_SOURCE_NONE;            // 非法来源：安全起见选 NONE
    }

    // 来源没有变化时保留现有数据，避免反复清掉刚收到的有效帧。
    if (input->selected == source)
    {
        return;
    }

    // 记录新选择；具体命令是否可用仍由 GetSafe 判断。
    input->selected = source;

    // 切换后丢弃该来源的旧命令，必须等它发来新的一帧
    if (source == REMOTE_SOURCE_PHONE)
    {
        input->phone = (RemoteState_t){0};
    }
    else if (source == REMOTE_SOURCE_HANDHELD)
    {
        input->handheld = (RemoteState_t){0};
    }
}

/**
 * @brief 保存指定来源最近一次通过校验的遥控命令及其接收时间。
 * @param input 遥控输入管理对象指针。
 * @param source 命令来源，只接受手机或自制手柄。
 * @param command 待保存的命令，三个摇杆轴均须在 -1.0f 到 1.0f 范围内。
 * @param now_ms 收到有效命令时的时间，单位为毫秒。
 * @return 参数和轴值有效时返回 true；否则返回 false，原状态不变。
 * @note 应在完整数据帧校验通过后调用，不能按单个串口字节更新有效时间。
 */
bool RemoteInput_Update(RemoteInput_t *input, RemoteSource_t source,
                        const RemoteCommand_t *command, uint32_t now_ms)
{
    // 空指针或任一摇杆轴超出 [-1, 1] 时拒绝更新，原状态保持不变。
    if (input == NULL || command == NULL ||
        !AxisValid(command->forward) ||
        !AxisValid(command->left) ||
        !AxisValid(command->turn)) {
        return false;
        }

    // 根据来源，让 state 指向 input 内对应的原状态，而不是复制一份状态。
    RemoteState_t *state;
    if (source == REMOTE_SOURCE_PHONE)
    {
        state = &input->phone;
    }
    else if (source == REMOTE_SOURCE_HANDHELD)
    {
        state = &input->handheld;
    }
    else
    {
        // NONE 或其他非法来源没有可保存的状态。
        return false;
    }

    // 将整份命令复制到选中的状态，并记录本次有效帧的接收时间。
    state->command = *command;
    state->last_valid_ms = now_ms;
    state->has_valid_frame = true;
    // true 只表示命令保存成功，不代表底盘已经开始运动。
    return true;
}

/**
 * @brief 读取当前来源的安全遥控命令。
 * @param input 遥控输入管理对象指针。
 * @param now_ms 当前时间，单位为毫秒。
 * @param timeout_ms 有效命令允许的最长间隔，单位为毫秒。
 * @return 仅在来源已选择、命令未超时且已使能并未请求停机时返回原命令；
 *         其他情况返回各字段均为零的命令。
 */
RemoteCommand_t RemoteInput_GetSafe(const RemoteInput_t *input,
                                    uint32_t now_ms, uint32_t timeout_ms)
{
    // 先准备全零命令；任何不安全情况都返回它。
    RemoteCommand_t zero = {0};

    // 无管理对象或超时阈值为零时，不允许输出运动命令。
    if (input == NULL || timeout_ms == 0U)
    {
        return zero;
    }

    // 只读取当前选中的来源；未选择来源时保持零输出。
    const RemoteState_t *state;
    if (input->selected == REMOTE_SOURCE_PHONE)
    {
        state = &input->phone;
    }
    else if (input->selected == REMOTE_SOURCE_HANDHELD)
    {
        state = &input->handheld;
    }
    else
    {
        return zero;
    }

    // 没有有效帧、帧已超时、未使能或请求停机时都不能使用旧命令。
    if (!state->has_valid_frame ||
        (uint32_t)(now_ms - state->last_valid_ms) > timeout_ms ||
        !state->command.enabled ||
        state->command.stop_requested)
    {
        return zero;
    }

    // 所有安全条件均满足，返回选中来源保存的完整命令。
    return state->command;
}
