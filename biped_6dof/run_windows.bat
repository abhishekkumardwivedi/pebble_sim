@echo off
cd /d %~dp0
python -m pip install -r requirements.txt
python biped_demo.py
pause
