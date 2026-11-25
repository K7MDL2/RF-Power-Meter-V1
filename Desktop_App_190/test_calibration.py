import ast
import math
import time
import types
import unittest
from pathlib import Path


def load_classes(path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    classes = [node for node in tree.body if isinstance(node, ast.ClassDef)
               and node.name in ("PowerCalibration", "Power_Data")]
    namespace = {"math": math, "time": time}
    exec(compile(ast.Module(body=classes, type_ignores=[]), str(path), "exec"), namespace)
    return namespace


class Widget:
    def __init__(self, value=""):
        self.value = value
        self.state = None

    def get(self):
        return self.value

    def config(self, **kwargs):
        self.state = kwargs.get("state", self.state)
        self.text = kwargs.get("text", "")

    def after(self, delay, callback, *args):
        self.pending = (callback, args)


class CalibrationCases:
    def setUp(self):
        self.namespace = load_classes(self.app_path)
        namespace = self.namespace
        self.commands = []
        self.errors = []
        namespace["meter_data"] = ["101", "170", "144MHz"]
        namespace["cal_response"] = (0, "", "", "", ())
        namespace["Send_Mtr_Cmds"] = lambda: types.SimpleNamespace(
            send_meter_cmd=lambda *args: self.commands.append(args))
        namespace["tkinter"] = types.SimpleNamespace(messagebox=types.SimpleNamespace(
            showerror=lambda *args: self.errors.append(args)))
        self.window = namespace["PowerCalibration"]()
        self.window.cal_band = None
        self.window.cal_points = {}
        self.window.P_Units = Widget("Watts")
        self.window.Pwr_Hi = Widget("100")
        self.window.Pwr_Lo = Widget("10")
        for name in ("Cal_Hi_btn", "Cal_Lo_btn", "Cal_Fwd_btn", "Cal_Ref_btn",
                     "Cal_HiV_F_Text", "Cal_HiV_R_Text", "Cal_LoV_F_Text",
                     "Cal_LoV_R_Text", "Cfg_Band_label"):
            setattr(self.window, name, Widget())

    def acknowledge(self, command, values):
        namespace = self.namespace
        previous = namespace["cal_response"][0]
        namespace["cal_response"] = (previous + 1, "161", "144MHz", command, values)
        callback, args = self.window.Cfg_Band_label.pending
        callback(*args)

    def test_watts_points_and_forward_calculation(self):
        self.window.Cal_Hi()
        self.assertEqual(self.commands[-1], ("79", "100.0", True))
        self.assertEqual(self.window.Cal_Fwd_btn.state, "disabled")
        self.acknowledge("79", ("1.00000", "0", "1.20000", "0"))
        self.window.Cal_Lo()
        self.assertEqual(self.commands[-1], ("77", "10.0", True))
        self.acknowledge("77", ("1.00000", "1.20000", "1.20000", "1.40000"))
        self.assertEqual(self.window.cal_points[True][0], 50.0)
        self.assertEqual(self.window.cal_points[False][0], 40.0)
        self.window.Cal_Fwd()
        self.assertEqual(self.commands[-1], ("75", "", True))

    def test_invalid_inputs_and_identical_voltages(self):
        self.window.Pwr_Hi.value = "nan"
        self.window.Cal_Hi()
        self.window.Pwr_Hi.value = "0"
        self.window.Cal_Hi()
        self.assertEqual(len(self.errors), 2)
        self.assertFalse(self.commands)
        self.window.cal_band = "144MHz"
        self.window.cal_points = {True: (50.0, 1.0, 2.0), False: (40.0, 1.0, 2.2)}
        self.window.Cal_Fwd()
        self.assertEqual(len(self.errors), 3)
        self.assertFalse(self.commands)

    def test_wrong_ack_and_timeout_do_not_mark_point_measured(self):
        self.window.Cal_Hi()
        self.namespace["cal_response"] = (1, "161", "144MHz", "77", ("1", "2", "3", "4"))
        callback, args = self.window.Cfg_Band_label.pending
        callback(*args[:-1], time.monotonic() - 1)
        self.assertEqual(self.window.cal_points, {})
        self.assertEqual(self.window.Cal_Fwd_btn.state, "disabled")
        self.assertTrue(self.errors)

    def test_dbm_zero_and_band_change(self):
        self.window.P_Units.value = "dBm"
        self.window.Pwr_Hi.value = "0"
        self.window.Cal_Hi()
        self.assertEqual(self.commands[-1], ("78", "0.0", True))
        self.acknowledge("78", ("1", "0", "2", "0"))
        self.namespace["meter_data"][2] = "432MHz"
        self.window.Cal_Fwd()
        self.assertEqual(len(self.commands), 1)
        self.assertTrue(self.errors)

    def test_meter_progress_messages_reach_pending_measurement(self):
        self.namespace["myRig_meter_ID"] = "101"
        self.namespace["cal_flag"] = 0
        self.window.Cal_Hi()
        self.namespace["Power_Data"]().get_power_data("101,161,144MHz,1.00000,0.00000,1.20000,0.00000,79\r\n")
        callback, args = self.window.Cfg_Band_label.pending
        callback(*args)
        self.assertEqual(self.window.cal_points[True], (50.0, 1.0, 1.2))
        self.assertEqual(self.window.Cal_HiV_F_Text.text, "F:1.00000VDC")

    def test_legacy_meter_progress_without_command_field(self):
        self.namespace["myRig_meter_ID"] = "101"
        self.namespace["cal_flag"] = 0
        self.window.Cal_Hi()
        self.namespace["Power_Data"]().get_power_data("101,161,144MHz,1.00000,0.00000,1.20000,0.00000\r\n")
        callback, args = self.window.Cfg_Band_label.pending
        callback(*args)
        self.assertEqual(self.window.cal_points[True], (50.0, 1.0, 1.2))


class Calibration190Tests(CalibrationCases, unittest.TestCase):
    app_path = Path(__file__).with_name("pyRFPowerMeter.py")


class CalibrationCabinetTests(CalibrationCases, unittest.TestCase):
    app_path = Path(__file__).parent.parent / "Desktop_App" / "pyRFPowerMeter.py"


class ConfigurationTests(unittest.TestCase):
    def test_instance_defaults_and_sensor_controls(self):
        for path, expected, sensor_controls in (
            (CalibrationCabinetTests.app_path,
             ("Cabinet", "100", "192.168.2.188", 7942, 7943, 2238, 2), True),
            (Calibration190Tests.app_path,
             ("K3 w/2M Amp", "101", "192.168.2.190", 7940, 7941, 2239, 1), False),
        ):
            with self.subTest(path=path):
                tree = ast.parse(path.read_text(encoding="utf-8"))
                keys = ("myRig", "myRig_meter_ID", "IPADDR_OF_METER",
                        "PORTNUM_OF_METER_LISTEN", "PORTNUM_OF_METER_SENDTO",
                        "WSJTX_UDP_PORT", "HIDE_POWER_INFO")
                values = {}
                for node in tree.body:
                    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
                        if node.targets[0].id in keys:
                            values[node.targets[0].id] = ast.literal_eval(node.value)
                self.assertEqual(tuple(values[key] for key in keys), expected)
                cfg = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "Cfg_Mtr")
                window = next(node for node in cfg.body if isinstance(node, ast.FunctionDef) and node.name == "Cfg_Window")
                found = any(isinstance(node, ast.Attribute) and node.attr == "Cal_Temp_btn"
                            and isinstance(node.ctx, ast.Store) for node in ast.walk(window))
                self.assertEqual(found, sensor_controls)


if __name__ == "__main__":
    unittest.main()
