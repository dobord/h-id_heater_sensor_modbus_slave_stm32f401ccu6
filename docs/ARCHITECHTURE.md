# Архитектура проекта

Проект: модбас-слейв датчиков температуры на STM32F401CCU6 с доступом по RS485 (Modbus RTU) и USB CDC (Modbus RTU over USB).

Важное примечание об источниках:
- Описание основано на анализе исходного кода репозитория. Некоторые выводы могут быть неполными.
- Для просмотра и уточнения по исходникам используйте поиск по коду GitHub:
  - DeviceMemoryModbus (поиск по коду): https://github.com/search?q=repo%3Adobord%2Fh-id_heater_sensor_modbus_slave_stm32f401ccu6+DeviceMemoryModbus&type=code
  - SystemShortRequestID (поиск по коду): https://github.com/search?q=repo%3Adobord%2Fh-id_heater_sensor_modbus_slave_stm32f401ccu6+SystemShortRequestID&type=code

## Аппаратная архитектура

Микроконтроллер: STM32F401CCU6 (Cortex‑M4F, 84 МГц)

Подсистемы и линии:
- Питание и отладка:
  - SWD: PA13 (SWDIO), PA14 (SWCLK)
- Датчики температуры:
  - Два NTC10K на АЦП:
    - SENSOR_T1: PA6 (ADC1_IN6)
    - SENSOR_T2: PA7 (ADC1_IN7)
  - Два PT1000 через MAX31865 по SPI2:
    - SPI2: PB13 (SCK), PB14 (MISO), PB15 (MOSI)
    - CS каналы: PB0 (CS1), PB1 (CS2)
- RS485:
  - USART2: PA2 (TX), PA3 (RX)
  - RS485_RSE (DE/RE): PA1 (GPIO Output)
- USB:
  - USB_OTG_FS: PA11 (DM), PA12 (DP)
- Память конфигурации (EEPROM):
  - AT24C32A по I2C1: PB6 (SCL), PB7 (SDA)
- Индикация:
  - LED1: PB2 (Red), LED2: PB10 (Green), LED3: PC13 (Blue)
- Дополнительно:
  - 74HC595 (управление регистрами/индикаторами): PA5 (_MR), PA8 (_STCP), PB13 (SHCP), PB15 (DS)

Тактовые настройки (из .ioc):
- HSE: 25 МГц, SYSCLK: 84 МГц, USB clock: 48 МГц (через PLL)

## Программная архитектура

Стек:
- HAL STM32CubeF4 (FW_F4 V1.27.0)
- FreeRTOS
- USB CDC класс устройства (для Modbus по USB)
- Драйверы: ADC+DMA, SPI2, I2C1, USART2
- Библиотека Modbus (конфиг: ENABLE_USB_CDC=1, ENABLE_USART_DMA=1)
- Собственные модули: логика датчиков, преобразование значений, управление регистровой картой

### Память устройства для Modbus

Структура DeviceMemoryModbus (разделяемая регистровая карта Modbus):
- Регистры 0–15: reserved_relay_out[16] (резерв, выровнено, чтобы «короткий порт» начинался с 16)
- Короткий порт:
  - short_response (addr 16)
  - short_request (addr 17)
  - short_data[hscc_short_io_length] (addr ≥ 18)
- Статусы внутренних датчиков: int_sens_status[length_of_ctl_reg_internal_sensors]
- Значения внутренних датчиков: int_sens_value[max_num_of_internal_sensors]
- Регистр управления устройством: dev_ctl (DeviceControlRegister_SensorTemperature, 16-бит битовое поле)
- Длинный порт:
  - long_response
  - long_request
  - long_data[hscc_long_io_length]

Константа DeviceMemoryModbusLength = sizeof(DeviceMemoryModbus)/2 — число 16‑бит слов в карте.

Назначение портов:
- Короткий порт (short_*) — синхронные короткие команды/ответы (версия, длина портов, тип ПО, длина карты, чтение/установка RS485 ID и т.п.).
- Длинный порт (long_*) — обмен «длинными» сообщениями/данными (например, блочные операции).

Регистр управления DeviceControlRegister_SensorTemperature:
- bit0 rs485_io — включение режима RS485 I/O (см. раздел Modbus API)
- bit2 led_red, bit3 led_green, bit4 led_blue — управление светодиодами
- bit6 debug_int_relay_to_led — отладочный режим индикации
- Остальные биты — зарезервированы

### Конфигурация в EEPROM (AT24C32A)

Формат заголовка конфигурации (ConfigMemory):
- marker (u32), version (u16), length (u16), software_type (u8), rs485_id (u8), header_crc (u16)

Карта EEPROM (enum HeaterSystemEepromMemory):
- 0–31: hsem_reserved_0
- 32: hsem_config_marker (u32)
- 36: hsem_config_version (u16)
- 38: hsem_config_length (u16)
- 40: hsem_config_software_type (u8)
- 41: hsem_config_rs485_id (u8)
- 42: hsem_config_header_crc (u16)
- 44: hsem_device_memory — бинарный снимок DeviceMemoryModbus (u8[sizeof(DeviceMemoryModbus)])

Назначение:
- Хранение ID RS485, типа ПО, валидатора заголовка и образа «оперативной» карты для восстановления.

### Инициализация и жизненный цикл

Порядок запуска (главное, по коду main.c):
1. Инициализация HAL, тактирования, GPIO, DMA, ADC1, I2C1, SPI2, USART2, RTC, USB_DEVICE.
2. Инициализация MAX31865 для двух каналов (SPI2 + PB0/PB1 CS).
3. Запуск ADC DMA на массив выборок.
4. Инициализация двух обработчиков Modbus:
   - USB CDC как MB_SLAVE, ID=1, u16regs указывает на DeviceMemoryModbus
   - RS485 (USART2, DMA) как MB_SLAVE, ID считывается из конфигурации (rs485_id), EN на PA1
5. Запуск Modbus (USB — ModbusStartCDC, RS485 — ModbusStart).
6. Инициализация FreeRTOS ядра и задач.

Обмен по USB CDC:
- Прием данных обрабатывается из прерываний USB CDC (CDC_Receive_FS), с уведомлением задачи Modbus.

Обновление датчиков:
- Периодическое чтение ADC (NTC10K) и MAX31865 (PT1000), вычисление температуры, обновление регистров int_sens_status / int_sens_value.

### Потоки данных

- Датчики → обработка (фильтрация/формулы) → обновление памяти DeviceMemoryModbus
- Внешний мастер (RS485 или USB CDC) → чтение/запись регистров (FC3/FC6/FC16) → короткие/длинные порты → выполнение запросов (версия, длины, конфигурация, управление)
- EEPROM ↔ загрузка/сохранение конфигурации и снимка карты

## Зависимости и конфигурация Modbus

Конфигурация (ModbusConfig.h):
- ENABLE_USB_CDC=1 — поддержка Modbus RTU over USB CDC
- ENABLE_USART_DMA=1 — DMA для USART
- MAX_BUFFER=128 — размер буфера
- MAX_M_HANDLERS=3 — число модбас-обработчиков

Два параллельных обработчика модбас:
- g_modbus_usb_cdc (USB CDC)
- g_modbus_rs485 (USART2 + RS485 EN)

Оба работают с общей картой регистров g_modbus_mem (DeviceMemoryModbus).

## Ключевые файлы

- Core/Src/main.c — инициализация, карта регистров, обработка коротких запросов
- Core/Inc/heater_modbus.h — типы, битовые поля регистров управления, идентификаторы сообщений
- Core/Inc/ModbusConfig.h — конфигурация библиотеки Modbus
- USB_DEVICE/App/usbd_cdc_if.c — обработчик USB CDC
- .ioc, .txt — конфигурация пинов и проекта STM32CubeMX