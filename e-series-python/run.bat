@echo off
echo === ${model_name} Python Samples ===
echo.
echo 可用案例:
$cases_list
echo.
set /p case_id="请输入案例ID运行 (如 servoJ): "
python samples\showcase_%case_id%.py
pause
