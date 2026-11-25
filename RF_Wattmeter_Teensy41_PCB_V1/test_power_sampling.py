import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


SKETCH = Path(__file__).with_name("RF_Wattmeter_Teensy41_PCB_V1.ino")
HEADER = SKETCH.with_suffix(".h")


def firmware_function(source, name):
    match = re.search(r"\b(?:void|float|bool) " + name + r"\([^)]*\)[^\n]*\n\{", source)
    if not match:
        raise ValueError(f"Function not found: {name}")
    depth, end = 1, match.end()
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


class PowerSamplingTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which("g++"), "g++ is required for the firmware sampling test")
    def test_peak_sampling_calibration_and_carrier_off(self):
        source = SKETCH.read_text(encoding="utf-8")
        header = HEADER.read_text(encoding="utf-8")
        definitions = "\n".join(
            re.search(r"^#define " + name + r"\s+[^\n]+", header, re.MULTILINE).group()
            for name in ("FWD_DISPLAY_CUTOFF", "FWD_PEAK_HOLD_MS", "PWR_ATTACK_ALPHA", "PWR_DECAY_ALPHA", "SWR_ATTACK_ALPHA",
                         "SWR_DECAY_ALPHA", "PWR_ZERO_HOLDOFF_MS",
                         "PTT_SWR_HOLDOFF_MS", "PTT_SWR_SPIKE_DELTA"))
        harness = r"""
#include <cassert>
#include <cmath>
#include <cstdint>
#define ADS1115_ADC
#define ADS1115_SINGLE_MODE
#define ADC_REF 0
#define ADC_FWD 1
#define NUMREADINGS 1
""" + definitions + r"""
float FwdPwr=0, RefPwr=0, SWRVal=0, FwdVal=0, RefVal=0;
float Fwd_dBm=0, Ref_dBm=0, SWR_Serial_Val=0, FwdPwr_last=0;
float Slope_F=-0.01274513f, Slope_R=-0.01274513f;
float Offset_F=0.62069817f, Offset_R=0.62069817f;
float CouplingFactor_Fwd=64, CouplingFactor_Ref=64, FwdPwr_Cutoff=9.9f;
float readings_Fwd[NUMREADINGS]={}, readings_Ref[NUMREADINGS]={};
float total_Fwd=0, total_Ref=0;
uint8_t readIndex_Fwd=0, readIndex_Ref=0;
uint32_t Timer_X00ms_InterruptCnt=0, Timer_X00ms_Last_AD=0, PTT_State_Change_ms=0;
uint32_t clock_ms=1000;
float fwd_samples[24], ref_samples[24];
int fwd_index=0, ref_index=0;
uint32_t millis() { return clock_ms; }
float readChannel(int channel) {
    if (channel == ADC_FWD) {
        assert(fwd_index < 24 && ref_index == fwd_index + 1);
        return fwd_samples[fwd_index++];
    }
    assert(ref_index == fwd_index);
    assert(ref_index < 24); return ref_samples[ref_index++];
}
void sendSerialData() {}
""" + "\n".join(firmware_function(source, name) for name in (
            "PeakDecayFilter", "PowerBelowCutoffExpired", "ApplyPowerZeroHoldoff", "ForwardPeakHoldFilter",
            "adRead", "PowerUpdateDue")) + r"""
float watts(float voltage, float slope, float offset) {
    return pow(10.0, (((voltage-offset)/slope+64)-30)/10);
}
void samples(float quiet, float peak, float reflected) {
    for (int i=0; i<24; ++i) { fwd_samples[i]=quiet; ref_samples[i]=reflected; }
    fwd_samples[8]=peak;
    fwd_index=ref_index=0;
}
int main() {
    uint32_t last_update=0;
    bool was_active=false;
    assert(!PowerUpdateDue(0, 999, last_update, was_active, 100, 1000, false));
    assert(PowerUpdateDue(500, 999, last_update, was_active, 100, 1000, false));
    assert(!PowerUpdateDue(500, 1098, last_update, was_active, 100, 1000, false));
    assert(PowerUpdateDue(500, 1099, last_update, was_active, 100, 1000, false));
    assert(PowerUpdateDue(0, 1100, last_update, was_active, 100, 1000, false));
    assert(!PowerUpdateDue(0, 2099, last_update, was_active, 100, 1000, false));
    assert(PowerUpdateDue(0, 2100, last_update, was_active, 100, 1000, false));
    assert(PowerUpdateDue(0, 2101, last_update, was_active, 100, 1000, true));
    last_update=UINT32_MAX-49; was_active=true;
    assert(PowerUpdateDue(100, 50, last_update, was_active, 100, 1000, false));
    assert(PowerUpdateDue(100, 50, last_update, was_active, 0, 1000, false));
    float held=ForwardPeakHoldFilter(0, 800, 100);
    assert(held==800);
    held=ForwardPeakHoldFilter(held, 100, 100+FWD_PEAK_HOLD_MS-1);
    assert(held==800);
    held=ForwardPeakHoldFilter(held, 100, 100+FWD_PEAK_HOLD_MS);
    assert(held<800 && held>100);
    held=ForwardPeakHoldFilter(held, 900, 600);
    assert(held==900);
    samples(0.90f, 0.68403f, 0.95f);
    adRead(false);
    assert(fwd_index==24 && ref_index==24);
    assert(fabs(FwdPwr-watts(0.68403f, Slope_F, Offset_F)) < 0.1);
    float average=(23*0.90f+0.68403f)/24;
    assert(fabs(FwdVal-average) < 0.00001);
    float fwd_average_watts=watts(average, Slope_F, Offset_F);
    float reflected_watts=watts(0.95f, Slope_R, Offset_R);
    float ratio=sqrt(reflected_watts/fwd_average_watts);
    float expected_swr=(1+ratio)/(1-ratio)*SWR_ATTACK_ALPHA;
    assert(fabs(SWRVal-expected_swr) < 0.001);
    clock_ms+=100;
    samples(0.90f, 0.68403f, 0.95f);
    adRead(true);
    assert(fabs(FwdVal-average) < 0.00001);
    Slope_F=-Slope_F; Offset_F=2-Offset_F;
    clock_ms+=100;
    samples(1.10f, 1.31597f, 0.95f);
    adRead(false);
    assert(fabs(FwdPwr-watts(1.31597f, Slope_F, Offset_F)) < 0.2);
    clock_ms+=100;
    samples(0.90f, 0.90f, 0.95f);
    adRead(false);
    clock_ms+=100;
    samples(0.90f, 0.90f, 0.95f);
    adRead(false);
    assert(FwdPwr>0);
    clock_ms+=50;
    samples(0.90f, 0.90f, 0.95f);
    adRead(false);
    assert(FwdPwr==0 && RefPwr==0 && SWRVal==0);

    Slope_F=-0.01274513f; Offset_F=0.62069817f;
    ApplyPowerZeroHoldoff(20, clock_ms);
    RefPwr=2; SWRVal=2;
    float low_voltage=Offset_F+Slope_F*(10*log10(5.0)+30-CouplingFactor_Fwd);
    clock_ms+=100;
    samples(low_voltage, low_voltage, 0.95f);
    adRead(false);
    assert(FwdPwr>4.9 && FwdPwr<5.1);
    clock_ms+=150;
    samples(low_voltage, low_voltage, 0.95f);
    adRead(false);
    assert(FwdPwr>4.9 && FwdPwr<5.1 && RefPwr==0 && SWRVal==0);
    float noise_voltage=Offset_F+Slope_F*(10*log10(0.5)+30-CouplingFactor_Fwd);
    clock_ms+=100;
    samples(noise_voltage, noise_voltage, 0.95f);
    adRead(false);
    assert(FwdPwr>0);
    clock_ms+=150;
    samples(noise_voltage, noise_voltage, 0.95f);
    adRead(false);
    assert(FwdPwr==0);
    float below_voltage=Offset_F+Slope_F*(10*log10(0.9)+30-CouplingFactor_Fwd);
    clock_ms+=100;
    samples(below_voltage, below_voltage, 0.95f);
    adRead(false);
    assert(FwdPwr==0);
    float above_voltage=Offset_F+Slope_F*(10*log10(1.1)+30-CouplingFactor_Fwd);
    clock_ms+=100;
    samples(above_voltage, above_voltage, 0.95f);
    adRead(false);
    assert(FwdPwr>1.05 && FwdPwr<1.15);
    clock_ms+=FWD_PEAK_HOLD_MS+1;
    samples(below_voltage, below_voltage, 0.95f);
    adRead(false);
    assert(FwdPwr==0);
}
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            cpp, executable = path / "sampling.cpp", path / "sampling"
            cpp.write_text(harness, encoding="utf-8")
            subprocess.run(["g++", "-std=c++17", str(cpp), "-o", str(executable)], check=True)
            subprocess.run([str(executable)], check=True)

    @unittest.skipUnless(shutil.which("g++"), "g++ is required for the Nextion timing test")
    def test_nextion_acknowledgements_do_not_delay_touch_handling(self):
        source = (SKETCH.parent / "nextion_patched" / "ITEADLIB_Arduino_Nextion_patched"
                  / "NexHardware.cpp").read_text(encoding="utf-8")
        harness = r"""
#include <cassert>
#include <cstdint>
#include <deque>
#define NEX_RET_EVENT_TOUCH_HEAD 0x65
#define page0_ID 0
#define page1_ID 1
#define page2_ID 2
#define page3_ID 3
#define page4_ID 4
#define toMain_ID 10
#define toConfig_ID 11
#define toSet1_ID 12
#define toPwrGraph_ID 13
#define toPowerCal_ID 14
#define dbSerialPrint(x) ((void)0)
#define dbSerialPrintln(x) ((void)0)
int pg=0, delay_count=0, events=0;
struct SerialMock {
    std::deque<uint8_t> bytes;
    int available() { return bytes.size(); }
    uint8_t read() { auto value=bytes.front(); bytes.pop_front(); return value; }
} nexSerial;
void delay(int ms) {
    assert(ms==10);
    ++delay_count;
    for (uint8_t b : {0, 11, 1, 0xff, 0xff, 0xff}) nexSerial.bytes.push_back(b);
}
struct NexTouch {
    static void iterate(NexTouch **, uint8_t page, uint8_t component, int state) {
        assert(page==0 && component==11 && state==1);
        ++events;
    }
};
""" + firmware_function(source, "nexLoop") + r"""
int main() {
    nexSerial.bytes={1, 0xff, 0xff, 0xff, 1, 0xff, 0xff, 0xff};
    nexLoop(nullptr);
    assert(delay_count==0 && events==0);
    nexSerial.bytes={0x65, 0, 11, 1, 0xff, 0xff, 0xff};
    nexLoop(nullptr);
    assert(delay_count==0 && events==1 && pg==1);
    nexSerial.bytes={0x65};
    nexLoop(nullptr);
    assert(delay_count==1 && events==2);
}
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            cpp, executable = path / "nextion.cpp", path / "nextion"
            cpp.write_text(harness, encoding="utf-8")
            subprocess.run(["g++", "-std=c++17", "-static-libstdc++", "-static-libgcc",
                            str(cpp), "-o", str(executable)], check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    unittest.main()
