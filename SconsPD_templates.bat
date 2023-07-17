cd %~dp0\godot-4.1-PD

scons platform=windows target=template_debug precision=double arch=x86_64
scons platform=windows target=template_release precision=double arch=x86_64

:: scons platform=windows target=template_debug precision=double arch=x86_32
:: scons platform=windows target=template_release precision=double arch=x86_32

pause