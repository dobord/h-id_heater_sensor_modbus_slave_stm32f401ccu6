# Modbus API

Полное описание карты регистров, протокольных параметров и примеров обмена.

Содержание:
- Параметры транспорта
- Пространство адресов и регистры
- Типы и единицы измерения
- Исключения и ошибки
- Тайминги RTU
- Примеры кадров
- Идентификация устройства
- Профиль совместимости с Heater Control Modbus Master (доб. раздел)
- Окно памяти DeviceMemoryModbus (Holding)

## Параметры транспорта

- Режим: Modbus RTU (Slave).
- Адрес ведомого (Slave ID): по умолчанию 1 (диапазон 1…247).
- UART параметры по умолчанию: 9600, 8N1 (настраивается).
- Поддерживаемые FC:
  - 01 Read Coils
  - 02 Read Discrete Inputs
  - 03 Read Holding Registers
  - 04 Read Input Registers
  - 05 Write Single Coil
  - 06 Write Single Register
  - 15 Write Multiple Coils
  - 16 Write Multiple Registers
  - 43/14 Read Device Identification (Optional)

CRC-16 (Modbus): полином 0xA001, начальное значение 0xFFFF, порядок байтов — LSB first.

USB-CDC (сервис/отладка): канал логирования и сервисных команд с ПК/приложения. Для обслуживания используется приложение [h-id_heater_gui](https://github.com/dobord/h-id_heater_gui).

## Пространство адресов и регистры

Примечание: адреса указаны в нотации Modbus (1‑based) и в скобках — 0‑based offset.

### Discrete Inputs (1xxxx) — Read‑Only, статус

- 10001 (0): SENSOR_OK — 1 = датчик(и) ОК, 0 = ошибка
- 10002 (1): OVER_TEMP — перегрев
- 10003 (2): UNDERVOLT — пониженное питание
- 10004 (3): CONFIG_DIRTY — есть несохраненные изменения
- 10005 (4): MODBUS_RX_ERR — была ошибка приема (сброс по чтению Input Reg «ERR_CLR»)

### Coils (0xxxx) — Read/Write, команды/флаги

- 00001 (0): MEAS_ENABLE — включить/выключить измерения
- 00002 (1): SAVE_CONFIG — запись конфигурации во FLASH (авто‑сброс в 0)
- 00003 (2): FACTORY_RESET — сброс к дефолтам (авто‑сброс в 0)
- 00004 (3): REBOOT — программная перезагрузка (авто‑сброс)

### Input Registers (3xxxx) — Read‑Only, измерения и телеметрия

- 30001 (0): TEMP_1 (0.01 °C) — int16
- 30002 (1): TEMP_2 (0.01 °C) — int16 (если нет второго датчика — 0x8000)
- 30003 (2): HEATER_PWR (0.1 %) — uint16
- 30004 (3): VCC_mV — uint16
- 30005 (4): MCU_TEMP (0.01 °C) — int16 (опционально)
- 30006 (5): STATUS_BITS — uint16 (битовая маска общих статусов)
- 30007 (6): FW_VERSION_MAJOR_MINOR — uint16 (MAJOR<<8 | MINOR)
- 30008 (7): FW_VERSION_PATCH — uint16
- 30009 (8): UPTIME_S — uint16 (секунды, кольцевой счетчик)
- 30010 (9): LAST_ERROR_CODE — uint16 (смотри «Исключения и ошибки»)

### Holding Registers (4xxxx) — Read/Write, конфигурация и управление

- 40001 (0): MB_ADDRESS — 1..247 (запись применится сразу)
- 40002 (1): MB_BAUD_CODE — 0:9600, 1:19200, 2:38400, 3:57600, 4:115200
- 40003 (2): MB_PARITY_STOP — биты: parity(0:None,1:Even,2:Odd), stop(1/2)
- 40004 (3): SAMPLE_PERIOD_MS — 10..1000
- 40005 (4): AVERAGING_WINDOW — 1..64
- 40006 (5): TEMP1_OFFSET_centiC — int16 (смещение, 0.01 °C)
- 40007 (6): TEMP2_OFFSET_centiC — int16
- 40008 (7): HEATER_PWR_LIMIT_tenths — 0..1000 (0..100.0 %)
- 40009 (8): CONFIG_FLAGS — битовые флаги (резерв)
- 40010 (9): FLASH_COMMIT_KEY — «магическое значение» для подтверждения сохранения (например, 0xA55A)
- 40011 (10): ERRORS_CLEAR — запись 1 очистит счетчики ошибок
- 40012 (11): DEVICE_MODE — 0:Normal, 1:Service
- 40013 (12): CTRL_FLAGS — управляющее слово-флаги (альтернатива Coil), биты определяются реализацией
- 40014 (13): SHORT_IO_BASE — базовый адрес «коротких команд» (только чтение; информативно)
- 40015 (14): UNUSED/RESERVED
- 40016 (15): UNUSED/RESERVED

Замечания:
- Сохранение во FLASH инициируется либо coil SAVE_CONFIG, либо записью FLASH_COMMIT_KEY = 0xA55A.
- Некоторые параметры могут применяться только после перезапуска UART (выполняется автоматически прошивкой).
- CTRL_FLAGS дублирует ключевые Coil-флаги битами 16‑битного слова для совместимости с маскированной записью (см. ниже).

## Типы и единицы измерения

- Температура: signed int16, шаг 0.01 °C, диапазон −327.68…+327.67 °C.
- Мощность/процент: uint16, шаг 0.1 % (0..1000).
- Напряжение: mV, uint16.
- Версия: два регистра (major.minor и patch).
- Битовые поля статусов: см. STATUS_BITS (документируется в коде/здесь).

## Исключения и ошибки

Коды исключений Modbus:
- 01 Illegal Function
- 02 Illegal Data Address
- 03 Illegal Data Value
- 04 Slave Device Failure
- 06 Slave Device Busy
- 08 Memory Parity Error (используется по необходимости)

LAST_ERROR_CODE (30010) отражает последнюю ошибку уровня устройства:
- 0: OK
- 1: SENSOR_DISCONNECT
- 2: SENSOR_SHORT
- 3: ADC_OOR (out-of-range)
- 4: OVER_TEMP
- 5: MODBUS_CRC
- 6: MODBUS_FORMAT
- 7: FLASH_CRC_FAIL
- 8+: зарезервировано

## Тайминги RTU

- Межсимвольная пауза: t1.5.
- Межкадровая пауза: t3.5 (обязательна для выделения кадров).
- Время ответа: см. PRD (T_response в зависимости от скорости UART).

## Примеры кадров (RTU)

Чтение 3 Input Registers начиная с 30001 (offset 0) — фактически FC=04, адресация 0‑based:
- Запрос: [Slave=0x01] [FC=0x04] [StartHi=0x00] [StartLo=0x00] [QtyHi=0x00] [QtyLo=0x03] [CRClo] [CRChi]
- Ответ: [0x01] [0x04] [0x06] [TEMP1_H] [TEMP1_L] [TEMP2_H] [TEMP2_L] [PWR_H] [PWR_L] [CRClo] [CRChi]

Запись одного Holding Register 40001 (адрес 0) значением 0x000A (Slave ID = 10):
- Запрос: [0x01] [0x06] [0x00] [0x00] [0x00] [0x0A] [CRClo] [CRChi]
- Ответ (эхо): [0x01] [0x06] [0x00] [0x00] [0x00] [0x0A] [CRClo] [CRChi]

Запись нескольких Holding (40004..40005): SAMPLE_PERIOD_MS=100, AVERAGING_WINDOW=8:
- Запрос: [0x01] [0x10] [0x00] [0x03] [0x00] [0x02] [0x04] [0x00] [0x64] [0x00] [0x08] [CRClo] [CRChi]
- Ответ: [0x01] [0x10] [0x00] [0x03] [0x00] [0x02] [CRClo] [CRChi]

## Идентификация устройства (опционально, 43/14)

- VendorName: "h-id"
- ProductCode: "heater-sensor"
- MajorMinorRevision: "MAJOR.MINOR.PATCH"
- Additional: серийный номер/датакод (если доступно)

## Профиль совместимости с Heater Control Modbus Master

Для корректной работы с dobord/h-id_heater_control_modbus_master_stm32f401ccu6 требуется:

1) Минимально необходимая карта для опроса:
- Мастер читает Input 30001..30010 (см. выше) для температур, версий и статусов.

2) Управляющее слово (CTRL_FLAGS):
- Дублирует ключевые управления из Coils (например, MEAS_ENABLE, SAVE_CONFIG, FACTORY_RESET) битами одного слова 40013 (offset 12).
- Позволяет мастеру атомарно изменять несколько флагов за одну транзакцию.

3) Short IO Service Area (вендорский сервис «маскированной записи»):
- Область: начиная с SHORT_IO_BASE (индикативный адрес — публикуется в 40014; фактический базовый адрес согласуется на уровне прошивки).
- Протокол (через FC=16 Write Multiple Registers):
  - Слово 0: OPCODE — значение, идентифицирующее операцию REPLACE_MASKED_BITS (конкретное значение уточняется; совместимо с мастером).
  - Слова 1..N: Параметры. Типичный формат:
    - Header: целевой регистр (dest_reg_addr), резерв/версия.
    - Data: MASK (битовая маска 16 бит), VALUE (новые биты).
  - Поведение: новое значение целевого регистра = (старое & ~MASK) | (VALUE & MASK).
  - Ответ: стандартный ответ FC=16.
- Назначение: безопасное обновление битовых флагов (например, CTRL_FLAGS) без гонок и с идемпотентностью при повторах.

Примечания:
- Конкретные значения OPCODE/структур следует синхронизировать с реализацией мастера (см. исходники мастера: ReplaceMaskedBitsHeader/ReplaceMaskedBitsData). При отсутствии — принять значения «по умолчанию» в паре «мастер–слейв».
- Повторная отправка одной и той же команды не должна приводить к нежелательным побочным эффектам (требуется идемпотентность).

## Окно памяти DeviceMemoryModbus (Holding)

Ниже описана логическая раскладка массива регистров «DeviceMemoryModbus», используемого для унифицированного доступа. Поля приводятся как смещения (0‑based) относительно базового адреса окна DMEM_BASE в пространстве Holding. Базовый адрес DMEM_BASE задаётся прошивкой и может отличаться; фиксирован только внутренний порядок полей.

typedef struct DeviceMemoryModbus {
    uint16_t reserved_relay_out[16];
    uint16_t short_response;   // offset = 16
    uint16_t short_request;    // offset = 17
    uint16_t short_data[hscc_short_io_length]; // offset = 18 ..
    uint16_t int_sens_status[length_of_ctl_reg_internal_sensors];
    uint16_t int_sens_value[max_num_of_internal_sensors];
    DeviceControlRegister_SensorTemperature dev_ctl;
    uint16_t long_response;
    uint16_t long_request;
    uint16_t long_data[hscc_long_io_length];
} DeviceMemoryModbus;

Карточка полей и протокол обмена:

- [0..15] reserved_relay_out[N] — регистры состояния внутренних реле.
  - Тип: uint16 (каждый — битовое поле/слово состояния вендорского назначения).
  - Доступ: R/W.
  - Назначение: унификация с мастер‑устройствами; может не использоваться в данном слейве.

- [16] short_response — код ответа короткого обмена.
  - Тип: uint16.
  - Доступ: R (ведомое обновляет после обработки запроса).
  - Семантика: 0 = OK; !=0 — код ошибки (конкретика кода определяется операцией).

- [17] short_request — код запроса короткого обмена (OPCODE).
  - Тип: uint16.
  - Доступ: W (мастер пишет OPCODE, затем параметры в short_data).

- [18..18+hscc_short_io_length-1] short_data — параметры/данные короткого обмена.
  - Тип: массив uint16.
  - Доступ: R/W (мастер пишет параметры; слейв заполняет итоговые данные при необходимости).
  - Пример: для REPLACE_MASKED_BITS: [dest_reg_addr, MASK, VALUE, …].

- [18+hscc_short_io_length .. + length_of_ctl_reg_internal_sensors-1] int_sens_status[] — статус внутренних датчиков.
  - Тип: массив uint16; битовые флаги наличия/ошибок по датчикам.
  - Доступ: R.

- Далее int_sens_value[] — значения внутренних датчиков.
  - Тип: массив uint16; формат значений соответствует прибору (например, температура в 0.01 °C).
  - Доступ: R.

- dev_ctl — регистр(ы) управления устройством (DeviceControlRegister_SensorTemperature).
  - Тип: одно или несколько 16‑битных слов.
  - Доступ: R/W.
  - Состав (ориентировочно):
    - Бит Enable измерений; бит FactoryReset; бит SaveConfig; биты режимов.
    - Точная разметка уточняется по определению DeviceControlRegister_SensorTemperature в коде.

- long_response / long_request / long_data[] — порт обмена длинными сообщениями (аналог short_*).
  - long_response — R, код завершения операции.
  - long_request — W, OPCODE длинной операции.
  - long_data[] — R/W, буфер параметров/данных длиной hscc_long_io_length слов.
  - Назначение: передача больших структур (калибровки, профили и т.п.).

Примечания к окну DMEM:
- Смещение short_response фиксировано = 16 независимо от версии устройства (унификация между устройствами).
- Константы hscc_short_io_length, hscc_long_io_length, length_of_ctl_reg_internal_sensors и max_num_of_internal_sensors определяются конфигурацией прошивки данного устройства и должны быть отражены в документации сборки.
- При проектировании карт регистров следует избегать перекрытия с основной конфигурационной областью Holding (40001..), если DMEM_BASE располагается в общем пространстве Holding.

## Примеры короткого обмена (Short IO) через FC=16

Атомарная «маскированная запись» битов в CTRL_FLAGS (адрес 40013 / offset 12 в Holding) через окно DMEM:
- Предпосылка: DMEM_BASE известно; short_response по смещению 16, short_request — 17, short_data — с 18.
- Запрос (FC=16, старт = DMEM_BASE+17, количество = 1 + K):
  - Пишем short_request = OPCODE_REPLACE_MASKED_BITS
  - Пишем в short_data: [dest_reg_addr=12, MASK=0x0001, VALUE=0x0001] — установка бита MEAS_ENABLE
- Ответ: стандартный FC=16 (эхо). После обработки слейв выставит short_response = 0 (OK).
