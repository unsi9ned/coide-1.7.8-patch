# Patch for CoIDE 1.7.8



## Что такое CooCox IDE

CooCox CoIDE - это свободная среда разработки программного обеспечения для Windows, основанная на Eclipse и GCC toolchain, предназначенная для разработки ПО микроконтроллеров архитектуры ARM. CooCox CoIDE является одним из самых простых и быстрых в плане установки, освоения и настройки решений, позволяющим даже начинающим пользователям добиваться в нем существенных результатов. К сожалению разработка данной IDE была прекращена, как и прекращено существование официального сайта [www.coocox.org](#editing-this-readme). Потому далее приведена короткая инструкция где скачать CooCox CoIDE версии 1.7.8 для Windows.

## Зачем нужен этот патч

Если Вы предпочитаете разработку ПО в Eclipse, не хотите углубляться в тонкости сборки SDK, настройку линкера и компилятора, а внешний вид и редактор IDE Keil, SEGGER Embedded Studio и т.д. кажется вам морально устаревшим, то CoIDE может быть хорошей альтернативой. Данный патч поможет добавить поддержку новых ARM-микроконтроллеров в среду CooCox CoIDE. Вот список микроконтроллеров, которые поддерживаются на данный момент:
* Microchip
	- ATSAMD21G18A
* Nordic Semiconductor
	- nRF52832

Патч включает в себя также драйверы программирования Flash для данных МК, драйвер библиотеки DW1000. 

Добавлена поддержка плат:
* [NodeMCU-BU01](https://hamlab.net/mcu/decawave/evalution-boards-dwm1000-bu01/#nodemcu-bu01)
* [UWB-Feather](https://hamlab.net/mcu/decawave/evalution-boards-dwm1000-bu01/#uwb-feather)
* [DWM1001-DEV](https://www.qorvo.com/products/p/DWM1001-DEV)

## Что необходимо скачать и установить

1. [CooCox CoIDE версии 1.7.8](https://github.com/unsi9ned/coide-1.7.8-patch.git) (md5: 7970ebfb154ac2de5305f10cd52324e3)
2. GCC Toolchain [gcc-arm-none-eabi-10.3-2021.10](https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-win32.exe) (можно и новее, но тестирование производилось на данной версии)
3. [Git for Windows](https://git-scm.com/downloads/win)

## Как применить данный патч
1. Установить CoIDE в каталог c:/CooCox/CoIDE
2. Запустить Git Bash и выполнить следующий набор команд:
```
cd c:/CooCox/CoIDE
git init
git remote add origin https://github.com/unsi9ned/coide-1.7.8-patch.git
git pull origin master
git checkout -f master
```

После выполнения данных действий будут модифицированы некоторые файлы среды CoIDE и добавлены новые компоненты, в том числе драйверы периферии из nRF5 SDK 12.1.0, ядро Arduino для ATSAMD21G18A, примеры и т.д.

## Как осуществлять программирование и отладку
1. Если у вас в наличии имеется J-Link, то можно использовать его для отладки и программирования flash. Среда CoIDE имеет все необходимые средства для работы с данным отладчиком.
2. Использовать плату BluePill или китайский ST-Link V2 прошитый программой [STM32F103C8T6_CMSIS-DAP_SWO](https://github.com/RadioOperator/STM32F103C8T6_CMSIS-DAP_SWO). Всю необходимую информацию по программированию, а также hex-файлы можно найти в репозитории [https://github.com/RadioOperator/STM32F103C8T6_CMSIS-DAP_SWO](https://github.com/RadioOperator/STM32F103C8T6_CMSIS-DAP_SWO). Если Вы выбрали данный вариант, то в настройках IDE необходимо указать Adapter: **CMSIS-DAP** Более ничего от пользователя не требуется, т.к. нужные алгоритмы программирования уже добавлены. Программирование flash осуществляется с помощью интегрированной в среду утилиты **CoFlash 1.4.8**

## Portable-версия
1. Скачать пропатченную стабильную версию [CoIDE-1.7.8](https://github.com/unsi9ned/coide-1.7.8-patch/releases/latest)
2. Распаковать в любой каталог (рекомендуется в корень раздела)
3. Запустить CoIDE.exe
