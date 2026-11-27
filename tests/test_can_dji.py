"""执行当前DJI发送源码，在字段写入和BSP调用处构造任务抢占，核对实际提交给HAL的报文。"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

HARDWARE = r'''
#ifndef TEST_CAN_HW_H
#define TEST_CAN_HW_H
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef struct { unsigned bus; } hcan_t;
typedef struct { uint32_t StdId, ExtId, IDE, RTR, DLC, TransmitGlobalTime; } CAN_TxHeaderTypeDef;
typedef struct { hcan_t *hcan; CAN_TxHeaderTypeDef tx_header; uint8_t tx_data[8]; } CanCtrlData_s;
typedef struct { unsigned id,type,mode; struct { int16_t value; } set; } Motor_s;
enum { CAN_ID_STD=0, CAN_RTR_DATA=0, DJI_M2006=1, DJI_M3508, DJI_M6020,
       DJI_CURRENT_MODE=10, DJI_VOLTAGE_MODE=11 };
static hcan_t hcan1={1},hcan2={2};
static void test_hook(unsigned point);
/* 只替换HAL，BSP仍执行当前磁盘函数。 */
static uint32_t HAL_CAN_GetTxMailboxesFreeLevel(hcan_t *can) {
    (void)can;test_hook(5);return 3;
}
static unsigned count,recorded_bus[2],recorded_id[2];
static uint8_t recorded_data[2][8];
static int HAL_CAN_AddTxMessage(hcan_t *can,CAN_TxHeaderTypeDef *header,uint8_t *data,uint32_t *mailbox) {
    if(count>=2||header->IDE!=CAN_ID_STD||header->RTR!=CAN_RTR_DATA||header->DLC!=8) abort();
    recorded_bus[count]=can->bus;recorded_id[count]=header->StdId;
    memcpy(recorded_data[count],data,8);++count;*mailbox=0;return 0;
}
static void delay_us(unsigned value) { (void)value; }
void CAN_SendTxMessage(CanCtrlData_s *frame);
#endif
'''

CASES = r'''
static unsigned selected_point,armed,scenario;
static const int16_t chassis[4]={-3000,-3000,3000,3000};
static const int16_t stopped[4]={0,0,0,0};
static const int16_t pitch[4]={0,0,321,0};
/* 模拟真实优先级允许的底盘任务被发射任务抢占；另查同优先级云台切换。 */
static void test_hook(unsigned point) {
    if(armed&&point==selected_point) {
        armed=0;
        if(scenario==0) CanCmdDjiMotor(2,0x200,0,0,0,0);
        else CanCmdDjiMotor(2,0x1ff,0,0,321,0);
    }
}
static int check_frame(unsigned index,unsigned bus,unsigned id,const int16_t *values) {
    int ok=recorded_bus[index]==bus&&recorded_id[index]==id;
    int16_t actual[4];
    for(unsigned i=0;i<4;++i) {
        actual[i]=(int16_t)(((uint16_t)recorded_data[index][2*i]<<8)|recorded_data[index][2*i+1]);
        if(actual[i]!=values[i]) ok=0;
    }
    if(!ok) printf("frame %u: requested CAN%u 0x%x [%d,%d,%d,%d], submitted CAN%u 0x%x [%d,%d,%d,%d]\n",
        index,bus,id,values[0],values[1],values[2],values[3],recorded_bus[index],recorded_id[index],
        actual[0],actual[1],actual[2],actual[3]);
    return ok;
}
int main(int argc,char **argv) {
    if(argc!=4) return 2;
    scenario=(unsigned)atoi(argv[1]);selected_point=(unsigned)atoi(argv[2]);
    unsigned direct=(unsigned)atoi(argv[3]);armed=selected_point!=0;
    unsigned bus=scenario==0?1:2,id=0x200;
    const int16_t *values=scenario==0?chassis:stopped;
    if(direct) MultipleMotorControl(bus==1?&hcan1:&hcan2,id,values[0],values[1],values[2],values[3]);
    else CanCmdDjiMotor(bus,id,values[0],values[1],values[2],values[3]);
    unsigned outer_index=0;
    int ok=1;
    if(selected_point) {
        if(armed||count!=2) return 3;
        ok=check_frame(0,2,scenario==0?0x200:0x1ff,scenario==0?stopped:pitch);
        outer_index=1;
    } else if(count!=1) return 4;
    ok=check_frame(outer_index,bus,id,values)&&ok;
    if(!ok) return 1;
    /* 非法总线不得产生发送；检查后续正常调用的字节边界。 */
    count=0;CanCmdDjiMotor(0,0x200,1,2,3,4);CanCmdDjiMotor(3,0x200,1,2,3,4);
    if(count) return 5;
    const int16_t edges[4]={-32768,32767,-1,0};
    CanCmdDjiMotor(1,0x1ff,edges[0],edges[1],edges[2],edges[3]);
    if(count!=1||!check_frame(0,1,0x1ff,edges)) return 6;
    return 0;
}
'''


def function_text(path, signature):
    """只抽取完整BSP函数体，替换其硬件依赖。"""
    source = path.read_text(encoding="utf-8")
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 1
    for end in range(opening + 1, len(source)):
        depth += (source[end] == "{") - (source[end] == "}")
        if depth == 0:
            return source[start:end + 1]
    raise RuntimeError("BSP函数体未闭合")


class DjiCanTest(unittest.TestCase):
    def test_nested_sends_keep_bus_id_and_payload(self):
        compiler = shutil.which("gcc")
        self.assertIsNotNone(compiler, "需要GCC执行实际发送源码")
        source = (ROOT / "application/robot_cmd/CAN_cmd_dji.c").read_text(encoding="utf-8")
        # 只在临时源码插入抢占钩子；字段赋值、报文打包与BSP实现保持磁盘原逻辑。
        assignments = [("CAN_CTRL_DATA.hcan = hcan;", 1),
                       ("CAN_CTRL_DATA.tx_header.StdId = std_id;", 2),
                       ("CAN_CTRL_DATA.tx_data[0] = (value_1 >> 8);", 3),
                       ("CAN_CTRL_DATA.tx_data[0] = (curr_1 >> 8);", 3),
                       ("CAN_CTRL_DATA.tx_data[3] = value_2;", 4),
                       ("CAN_CTRL_DATA.tx_data[3] = curr_2;", 4)]
        for statement, point in assignments:
            source = source.replace(statement, statement + f"\n    test_hook({point});")
        # 防止后续实现变动导致场景静默失去注入点。
        for point in range(1, 5):
            self.assertIn(f"test_hook({point});", source)
        bsp = function_text(ROOT / "bsp/boards/bsp_can.c", "void CAN_SendTxMessage(")
        with tempfile.TemporaryDirectory(prefix="dji-can-") as directory:
            work = Path(directory)
            (work / "test_hw.h").write_text(HARDWARE, encoding="utf-8")
            for header in ("CAN_cmd_dji.h", "bsp_delay.h"):
                (work / header).write_text('#include "test_hw.h"\n', encoding="utf-8")
            bridge = work / "can_test.c"
            bridge.write_text(source + "\n" + bsp + "\n" + CASES, encoding="utf-8")
            executable = work / "can_test.exe"
            result = subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror",
                                     "-I", str(work), str(bridge), "-o", str(executable)],
                                    capture_output=True, text=True, errors="replace", timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            for scenario in (0, 1):
                for point in range(6):
                    for direct in (0, 1):
                        with self.subTest(scenario=scenario, point=point, direct=direct):
                            result = subprocess.run([str(executable), str(scenario), str(point), str(direct)],
                                                    capture_output=True, text=True, errors="replace", timeout=10)
                            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main(verbosity=2)
