#!/usr/bin/env bash
# 在电脑上编译并运行全部单元测试：小车 Test/ 和手柄 Handheld/Test/
# 用法：在仓库根目录运行 bash tools/run_host_tests.sh
# 需要 gcc 和 ar；产物放在 build/host_tests/，不影响固件构建
set -euo pipefail

cd "$(dirname "$0")/.."
OUT=build/host_tests
rm -rf "$OUT"
mkdir -p "$OUT/obj"

# 直接操作 HAL 或寄存器的源文件只能在单片机上编译，测试不需要它们
HW_ONLY=(
  Application/remote_uart.c
  BSP/encoder.c
  BSP/i2c_bus_hal.c
  BSP/motor.c
  BSP/safety.c
  BSP/servo.c
  Common/NRF24/NRF24_port.c
  Common/NRF24/nrf_link.c
)

INC=(-IApplication -IAlgorithm -IProtocol -IBSP -ICommon/Protocol -ICommon/NRF24 -IHandheld/Application)
CFLAGS=(-std=c11 -Wall -Wextra)

# 把可在电脑上编译的源文件打成静态库，每个测试只链接它用到的部分
for src in Application/*.c Algorithm/*.c Protocol/*.c BSP/*.c Common/*/*.c \
           Handheld/Application/hh_logic.c Handheld/Application/hh_input.c; do
  skip=0
  for hw in "${HW_ONLY[@]}"; do
    [[ "$src" == "$hw" ]] && skip=1
  done
  [[ $skip -eq 1 ]] && continue
  gcc "${CFLAGS[@]}" "${INC[@]}" -c "$src" -o "$OUT/obj/$(echo "$src" | tr / _).o"
done
ar rcs "$OUT/libhost.a" "$OUT"/obj/*.o

failed=0
for test in Test/test_*.c Handheld/Test/test_*.c; do
  name=$(basename "$test" .c)
  gcc "${CFLAGS[@]}" "${INC[@]}" "$test" "$OUT/libhost.a" -lm -o "$OUT/$name"
  if "$OUT/$name" > "$OUT/$name.log" 2>&1; then
    echo "通过  $name"
  else
    echo "失败  $name"
    cat "$OUT/$name.log"
    failed=1
  fi
done

exit $failed
