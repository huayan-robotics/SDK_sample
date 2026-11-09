#!/bin/bash
echo "=== ${model_name} Python Samples ==="
echo
echo "可用案例:"
$cases_list
echo
read -p "请输入案例ID运行 (如 servoJ): " case_id
python samples/showcase_${case_id}.py
