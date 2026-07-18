"""Static contracts for the optional SmartCar Volcengine TTS consumer."""
import ast
from pathlib import Path
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src" / "smartcar_speech"
CONFIG = PACKAGE / "config" / "speech.yaml"
LAUNCH = PACKAGE / "launch" / "smartcar_speech.launch.py"
NODE = PACKAGE / "smartcar_speech" / "speech_node.py"
CORE = PACKAGE / "smartcar_speech" / "speech_core.py"
CLIENT = PACKAGE / "smartcar_speech" / "volcengine_tts.py"
PACKAGE_XML = PACKAGE / "package.xml"


def launch_default(source, name):
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if not (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "DeclareLaunchArgument"
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and node.args[0].value == name
        ):
            continue
        for keyword in node.keywords:
            if keyword.arg == "default_value" and isinstance(
                keyword.value, ast.Constant
            ):
                return keyword.value.value
    raise AssertionError(f"launch argument {name!r} not found")


class SpeechLaunchContractTests(unittest.TestCase):
    def test_speech_is_disabled_by_default_and_consumes_only_output_text(self):
        config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        params = config["speech_node"]["ros__parameters"]
        self.assertIs(params["enabled"], False)
        self.assertEqual(params["input_topic"], "/smartcar/output/speech")
        self.assertEqual(params["status_topic"], "/smartcar/speech/status")
        self.assertEqual(
            params["api_url"],
            "https://openspeech.bytedance.com/api/v1/tts",
        )
        self.assertEqual(params["app_id_env"], "VOLCENGINE_TTS_APP_ID")
        self.assertEqual(
            params["access_token_env"],
            "VOLCENGINE_TTS_ACCESS_TOKEN",
        )
        self.assertEqual(params["player_argv"][0], "ffplay")
        self.assertIn("-nodisp", params["player_argv"])
        self.assertIn("-autoexit", params["player_argv"])
        self.assertIn("{audio_file}", params["player_argv"])
        self.assertEqual(params["max_text_bytes"], 1024)
        self.assertEqual(
            launch_default(LAUNCH.read_text(encoding="utf-8"), "enabled"),
            "false",
        )

    def test_credentials_are_environment_only_and_player_has_no_shell(self):
        config_source = CONFIG.read_text(encoding="utf-8")
        node_source = NODE.read_text(encoding="utf-8")
        client_source = CLIENT.read_text(encoding="utf-8")
        core_source = CORE.read_text(encoding="utf-8")
        self.assertNotIn("Authorization", config_source)
        self.assertIn("os.environ.get", node_source)
        self.assertIn('"Authorization": "Bearer;"', client_source)
        self.assertIn("shell=False", core_source)
        self.assertNotIn("os.system", core_source)
        self.assertNotIn("cmd_vel", node_source + core_source + client_source)

    def test_package_declares_ros_runtime_dependencies(self):
        source = PACKAGE_XML.read_text(encoding="utf-8")
        for dependency in ("launch", "launch_ros", "rclpy", "std_msgs"):
            self.assertIn(f"<exec_depend>{dependency}</exec_depend>", source)


if __name__ == "__main__":
    unittest.main()
