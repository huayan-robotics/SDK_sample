@echo off
echo === Building HY7 C# Samples (dotnet) ===
echo [BUILD] showcase_stateMachineManage
cd samples\showcase_stateMachineManage && dotnet build -c Release || exit /b 1
cd ..\..
echo [BUILD] showcase_robotErrorReset
cd samples\showcase_robotErrorReset && dotnet build -c Release || exit /b 1
cd ..\..
echo [BUILD] showcase_servoJ
cd samples\showcase_servoJ && dotnet build -c Release || exit /b 1
cd ..\..
echo [BUILD] showcase_servoP
cd samples\showcase_servoP && dotnet build -c Release || exit /b 1
cd ..\..
echo [BUILD] showcase_servoJPro
cd samples\showcase_servoJPro && dotnet build -c Release || exit /b 1
cd ..\..
echo [BUILD] showcase_servoPPro
cd samples\showcase_servoPPro && dotnet build -c Release || exit /b 1
cd ..\..
echo [BUILD] showcase_speedJ
cd samples\showcase_speedJ && dotnet build -c Release || exit /b 1
cd ..\..
echo [BUILD] showcase_speedL
cd samples\showcase_speedL && dotnet build -c Release || exit /b 1
cd ..\..
echo [BUILD] showcase_motion
cd samples\showcase_motion && dotnet build -c Release || exit /b 1
cd ..\..
echo [BUILD] showcase_motionrel
cd samples\showcase_motionrel && dotnet build -c Release || exit /b 1
cd ..\..
echo [BUILD] showcase_jog
cd samples\showcase_jog && dotnet build -c Release || exit /b 1
cd ..\..
echo [BUILD] showcase_motionStatusManagement
cd samples\showcase_motionStatusManagement && dotnet build -c Release || exit /b 1
cd ..\..
echo [OK] 构建成功
pause
