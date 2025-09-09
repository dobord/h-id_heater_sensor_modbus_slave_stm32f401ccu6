# Heater Sensor Modbus Slave (STM32F401CCU6)

Ведомое устройство Modbus RTU на базе STM32F401CCU6 для измерения температур и публикации телеметрии/статусов по шине RS‑485 (и/или сервисный USB‑CDC). Проект использует STM32Cube HAL, без RTOS (superloop + ISR/DMA).

- Основные возможности:
  - Измерение одной или нескольких температур (NTC/внутренние/внешние датчики).
  - Доступ к данным через Modbus: Input/Holding/Coils/Discrete Inputs.
  - Конфигурирование адреса/скорости/паритета, периода измерений, фильтрации, калибровок.
  - Диагностика: статусные флаги, счетчики ошибок, коды последней ошибки.
  - Сохранение настроек во FLASH, сброс к заводским.

## Документация

- Архитектура: [ARCHITECTURE.md](docs/ARCHITECTURE.md)
- Требования продукта (PRD): [PRD.md](docs/PRD.md)
- Системные требования: [SYSTEM_REQUIREMENTS.md](docs/SYSTEM_REQUIREMENTS.md)
- Modbus API и карта регистров: [MODBUS-API.md](docs/MODBUS-API.md)

Примечание: ссылки предполагают перенос документации в папку docs (PR #3). Если вы просматриваете этот PR до мержа #3, файлы могут временно лежать в корне.

## Быстрый старт

1) Сборка и прошивка
- Открыть проект в STM32CubeIDE (рекомендуется 1.14+).
- Конфигурация хранится в файле `.ioc`: h-id_heater_sensor_modbus_slave_stm32f401ccu6.ioc.
- Собрать и прошить целевой МК STM32F401CCU6.

2) Подключение интерфейсов
- RS‑485: трансивер (например, MAX3485/ADM3485), терминаторы 120 Ω на концах шины, корректный bias.
- DE/RE управляется отдельным GPIO (см. .ioc).
- USB‑CDC (опционально) — для отладки/сервиса.

3) Параметры по умолчанию
- Modbus RTU: адрес 1, 9600 бод, 8N1.
- Период измерений: 100 мс.
- Усреднение: окно 8.
- Все параметры настраиваются через Holding‑регистры (см. [Modbus API](docs/MODBUS-API.md)).

## Совместимость и инструменты

- Совместимый ведущий (master): [h-id_heater_control_modbus_master_stm32f401ccu6](https://github.com/dobord/h-id_heater_control_modbus_master_stm32f401ccu6).
- Настольное приложение/GUI (лог/сервис через USB‑CDC): [h-id_heater_gui](https://github.com/dobord/h-id_heater_gui).

## Структура проекта

- Core/, Drivers/, Middlewares/, USB_DEVICE/ (если включено CDC), startup/
- Файл линкера: `STM32F401CCUx_FLASH.ld`
- Файл конфигурации CubeMX: `h-id_heater_sensor_modbus_slave_stm32f401ccu6.ioc`
- Документация: `docs/`

## Основные ссылки

- Карта регистров, функции и примеры кадров: [Modbus API](docs/MODBUS-API.md)
- Архитектура прошивки и модули: [ARCHITECTURE.md](docs/ARCHITECTURE.md)
- Функциональные/нефункциональные требования: [PRD.md](docs/PRD.md)
- Аппаратные/системные требования: [SYSTEM_REQUIREMENTS.md](docs/SYSTEM_REQUIREMENTS.md)

## Обратная связь

Вопросы и предложения — через GitHub Issues и Pull Requests в этом репозитории.