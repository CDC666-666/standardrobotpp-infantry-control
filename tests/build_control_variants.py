"""复制工程到独立输出目录，验证 VT13 控车、旧控车及 UART2 的三种用途。"""
import argparse
import json
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MDK = ROOT / "MDK-ARM"
OUTPUT = MDK / "vt13-control-verification"
VARIANTS = {
    "vt13": [],
    "vt13-legacy-input": ["ROBOT_REMOTE_SOURCE=0"],
    "vofa": ["UART2_ROLE=0"],
    "legacy": ["UART2_ROLE=2"],
}


def prepare(variant):
    project = ET.parse(MDK / "standard_robot.uvprojx")
    for tag in ("FilePath", "ScatterFile", "RegisterFile", "SFDFile"):
        for node in project.iter(tag):
            # 只展开具体文件路径，保留 Keil 内部变量表达式。
            if node.text and not node.text.startswith("$$"):
                node.text = str((MDK / node.text).resolve())
    for node in project.iter("IncludePath"):
        # 只有配置了包含目录时才逐项转换为独立工程可用的绝对路径。
        if node.text:
            node.text = ";".join(str((MDK / value).resolve())
                                 for value in node.text.split(";") if value)
    directory = OUTPUT / variant
    directory.mkdir(parents=True, exist_ok=True)
    for tag in ("OutputDirectory", "ListingPath"):
        for node in project.iter(tag):
            node.text = str(directory) + "\\"
    defines = project.find(".//TargetArmAds/Cads/VariousControls/Define")
    assert defines is not None
    defines.text = ",".join([defines.text or "", *VARIANTS[variant]])
    path = OUTPUT / f"{variant}.uvprojx"
    project.write(path, encoding="utf-8", xml_declaration=True)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uv4", type=Path,
                        default=Path("D:/Robomaster/App/keil5/UV4/UV4.exe"))
    parser.add_argument("--variant", choices=VARIANTS, nargs="+", default=list(VARIANTS))
    args = parser.parse_args()
    results = []
    for variant in args.variant:
        project = prepare(variant)
        log = OUTPUT / f"keil-{variant}.log"
        process = subprocess.run([str(args.uv4), "-r", str(project), "-t", "standard_robot",
                                  "-j0", "-o", str(log)], cwd=MDK,
                                 creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        content = log.read_text(encoding="utf-8", errors="replace")
        counts = re.findall(r"- (\d+) Error\(s\), (\d+) Warning\(s\)\.", content)
        # 必须有最终摘要且零错误；退出码只接受成功或仅告警。
        passed = bool(counts) and counts[-1][0] == "0" and process.returncode in (0, 1)
        result = {"variant": variant, "passed": passed, "exit_code": process.returncode,
                  "errors": int(counts[-1][0]) if counts else None,
                  "warnings": int(counts[-1][1]) if counts else None, "log": str(log)}
        results.append(result)
        print(json.dumps(result, ensure_ascii=False), flush=True)
    (OUTPUT / "build-results.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
    # 任一配置失败都返回非零，不能用其他成功配置掩盖失败。
    if not all(result["passed"] for result in results):
        raise SystemExit(1)


# 直接运行时按所选配置生成独立工程并执行 Keil 构建。
if __name__ == "__main__":
    main()
