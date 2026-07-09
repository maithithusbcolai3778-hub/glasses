from pathlib import Path

html_path = Path("tools/imu_dashboard.html")
out_path = Path("main/imu_html.h")
html = html_path.read_text(encoding="utf-8")

escaped = html.replace("\\", r"\\").replace('"', r'\"').replace("\n", r"\n")

content = (
    "#pragma once\n"
    "/* Auto-generated from imu_dashboard.html. Do not edit manually. */\n"
    'static const char IMU_DASHBOARD_HTML[] = "' + escaped + '";\n'
)
out_path.write_text(content, encoding="utf-8")
print(f"generated {out_path} ({len(html)} bytes)")
