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

```c
typedef struct DeviceMemoryModbus {
    uint16_t reserved_relay_out[16];                              // offset = 0-15
    uint16_t short_response;                                      // offset = 16 (FIXED)
    uint16_t short_request;                                       // offset = 17
    uint16_t short_data[9];                                       // offset = 18-26 (hscc_short_io_length = 9)
    uint16_t int_sens_status[1];                                  // offset = 27 (length_of_ctl_reg_internal_sensors = 1)
    uint16_t int_sens_value[7];                                   // offset = 28-34 (max_num_of_internal_sensors = 7)
    DeviceControlRegister_SensorTemperature dev_ctl;             // offset = 35
    uint16_t long_response;                                       // offset = 36
    uint16_t long_request;                                        // offset = 37  
    uint16_t long_data[66];                                       // offset = 38-103 (hscc_long_io_length = 2 + MAX_BUFFER/2 = 66)
} DeviceMemoryModbus;
```

**Константы структуры:**
- `hscc_short_io_length = 9` — размер буфера коротких команд
- `hscc_long_io_length = 66` — размер буфера длинных команд (2 + MAX_BUFFER/2, MAX_BUFFER=128)
- `max_num_of_internal_sensors = 7` — количество внутренних датчиков (vref, vbat, tcpu, t1, t2, t3, t4)
- `length_of_ctl_reg_internal_sensors = 1` — размер статусного регистра датчиков ((7+15)/16)

**Внутренние датчики:**
- `isid_vref` (0): Опорное напряжение [voltage]
- `isid_vbat` (1): Напряжение батареи часов [voltage]  
- `isid_tcpu` (2): Температура CPU [temperature]
- `isid_t1` (3): Датчик 1 [temperature]
- `isid_t2` (4): Датчик 2 [temperature]
- `isid_t3` (5): Датчик 3 [temperature]
- `isid_t4` (6): Датчик 4 [temperature]

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
- Константы определяются конфигурацией прошивки:
  - `hscc_short_io_length = 9` — размер буфера коротких команд
  - `hscc_long_io_length = 66` — размер буфера длинных команд  
  - `max_num_of_internal_sensors = 7` — количество внутренних датчиков
  - `length_of_ctl_reg_internal_sensors = 1` — размер статусного регистра датчиков
- При проектировании карт регистров следует избегать перекрытия с основной конфигурационной областью Holding (40001..), если DMEM_BASE располагается в общем пространстве Holding.
- Общий размер DeviceMemoryModbus: 104 слова (208 байт).

## Справочник команд Short IO (Короткие команды)

Команды Short IO выполняются через запись в `short_request` с параметрами в `short_data[]`. Результат возвращается в `short_response` и `short_data[]`.

**Протокол выполнения:**
1. Мастер записывает команду в `short_request` (offset 17)
2. Мастер записывает количество аргументов в `short_data[0]` (offset 18)
3. Мастер записывает аргументы в `short_data[1..N]` (offset 19+)
4. Устройство обрабатывает команду через `handle_short_request()`
5. Устройство записывает код команды в `short_response` (offset 16)
6. Устройство записывает количество слов результата в `short_data[0]`
7. Устройство записывает результат в `short_data[1..M]`

### Системные команды Short IO (SystemShortRequestID)

| ID | Команда | Описание | Аргументы | Возврат |
|----|---------|----------|-----------|---------|
| 0 | `ssr_none` | Нет операции | - | - |
| 1 | `ssr_get_short_io_version` | Версия Short IO протокола | - | [1] версия (1) |
| 2 | `ssr_get_short_io_length` | Размер буфера Short IO | - | [1] размер (9) |
| 3 | `ssr_get_long_io_version` | Версия Long IO протокола | - | [1] версия (1) |
| 4 | `ssr_get_long_io_length` | Размер буфера Long IO | - | [1] размер (66) |
| 5 | `ssr_get_software_type` | Тип программного обеспечения | - | [1] тип |
| 6 | `ssr_get_device_memory_modbus_length` | Размер DeviceMemoryModbus | - | [1] размер в словах |

### RS485 и идентификация (50-86)

| ID | Команда | Описание | Аргументы | Возврат |
|----|---------|----------|-----------|---------|
| 50 | `ssr_get_rs485_id` | Получить RS485 адрес | - | [1] текущий адрес |
| 51 | `ssr_set_rs485_id` | Установить RS485 адрес | [1] новый адрес | [1] код ошибки |
| 52 | `ssr_get_devid` | Получить Device ID | - | [2] HAL_GetDEVID() |
| 53 | `ssr_get_revid` | Получить Revision ID | - | [2] HAL_GetREVID() |
| 54 | `ssr_get_uid` | Получить Unique ID | - | [6] HAL_GetUIDw0/1/2() |

### EEPROM операции (90-92)

| ID | Команда | Описание | Аргументы | Возврат |
|----|---------|----------|-----------|---------|
| 90 | `ssr_eeprom_factory_defaults` | Восстановить заводские настройки | - | [1] код результата |
| 91 | `ssr_eeprom_load_config` | Загрузить конфигурацию | - | [1] код результата |
| 92 | `ssr_eeprom_save_config` | Сохранить конфигурацию | - | [1] код результата |

### Адреса в DeviceMemoryModbus (100-141)

| ID | Команда | Описание | Аргументы | Возврат |
|----|---------|----------|-----------|---------|
| 100 | `ssr_get_addr_long_io` | Адрес long_response | - | [1] offset/2 |
| 101 | `ssr_get_addr_dev_ctl` | Адрес dev_ctl | - | [1] offset/2 |
| 102 | `ssr_get_addr_common_ctl` | Адрес common_ctl | - | [1] offset/2 |
| 103 | `ssr_get_addr_heater_ctl` | Адрес heater_ctl | - | [1] offset/2 |
| 104 | `ssr_get_addr_heater_param` | Адрес heater_param | - | [1] offset/2 |
| 105 | `ssr_get_addr_heater_counter_q` | Адрес heater_counter_q | - | [1] offset/2 |
| 110 | `ssr_get_addr_int_sens_status` | Адрес int_sens_status | - | [1] offset/2 |
| 111 | `ssr_get_addr_int_sens_value` | Адрес int_sens_value | - | [1] offset/2 |
| 120 | `ssr_get_addr_sens_addr` | Адрес sens_addr | - | [1] offset/2 |
| 121 | `ssr_get_addr_sens_desc` | Адрес sens_desc | - | [1] offset/2 |
| 122 | `ssr_get_addr_sens_enable` | Адрес sens_enable | - | [1] offset/2 |
| 123 | `ssr_get_addr_sens_status` | Адрес sens_status | - | [1] offset/2 |
| 124 | `ssr_get_addr_sens_value` | Адрес sens_value | - | [1] offset/2 |
| 130 | `ssr_get_addr_relay_ctl` | Адрес relay_ctl | - | [1] offset/2 |
| 131 | `ssr_get_addr_relay_status` | Адрес relay_status | - | [1] offset/2 |
| 132 | `ssr_get_addr_relay_value` | Адрес relay_value | - | [1] offset/2 |
| 140 | `ssr_get_addr_display_output_control` | Адрес display_output_control | - | [1] offset/2 |
| 141 | `ssr_get_addr_display_variable` | Адрес display_variable | - | [1] offset/2 |

### Максимальные значения (200-206)

| ID | Команда | Описание | Аргументы | Возврат |
|----|---------|----------|-----------|---------|
| 200 | `ssr_get_max_num_of_internal_sensors` | Макс. кол-во внутренних датчиков | - | [1] значение (7) |
| 201 | `ssr_get_max_num_of_internal_relays` | Макс. кол-во внутренних реле | - | [1] значение (8) |
| 202 | `ssr_get_max_num_of_sensors` | Макс. кол-во датчиков | - | [1] значение |
| 203 | `ssr_get_max_num_of_relays` | Макс. кол-во реле | - | [1] значение |
| 204 | `ssr_get_max_num_of_heater_ctl` | Макс. кол-во heater_ctl | - | [1] значение |
| 205 | `ssr_get_max_num_of_common_ctl` | Макс. кол-во common_ctl | - | [1] значение |
| 206 | `ssr_get_max_num_of_display_variable` | Макс. кол-во display_variable | - | [1] значение |

### Битовые операции (300)

| ID | Команда | Описание | Аргументы | Возврат |
|----|---------|----------|-----------|---------|
| 300 | `ssr_replace_masked_bits` | Маскированная запись битов | ReplaceMaskedBitsHeader + ReplaceMaskedBitsData[] | - |

**Структура ReplaceMaskedBitsHeader:**
```c
typedef struct {
    uint16_t reg_addr_start;  // Начальный адрес в DeviceMemoryModbus
    uint16_t num;             // Количество регистров для обработки
} ReplaceMaskedBitsHeader;
```

**Структура ReplaceMaskedBitsData:**
```c
typedef struct {
    uint16_t mask;   // Битовая маска
    uint16_t data;   // Новые биты
} ReplaceMaskedBitsData;
```

### RTC операции (400-434)

| ID | Команда | Описание | Аргументы | Возврат |
|----|---------|----------|-----------|---------|
| 400 | `ssr_rtc0_read_date_time` | Чтение даты/времени RTC0 | - | [4] RtcDataTime |
| 401 | `ssr_rtc0_write_date_time` | Запись даты/времени RTC0 | [4] RtcDataTime | - |
| 420 | `ssr_rtc1_read_date_time` | Чтение даты/времени RTC1 | - | [4] RtcDataTime |
| 421 | `ssr_rtc1_write_date_time` | Запись даты/времени RTC1 | [4] RtcDataTime | - |
| 422 | `ssr_rtc1_read_calibr` | Чтение калибровки RTC1 | - | [1] значение |
| 423 | `ssr_rtc1_write_calibr` | Запись калибровки RTC1 | [1] значение | - |
| 424 | `ssr_rtc1_read_regs` | Чтение регистров RTC1 | - | [N] регистры |
| 425 | `ssr_rtc1_write_regs` | Запись регистров RTC1 | [N] регистры | - |
| 426 | `ssr_rtc1_copy_to_rtc0` | Копирование RTC1 → RTC0 | - | - |

**Структура RtcDataTime:**
```c
typedef struct {
    uint8_t Year;      // Год (0-99)
    uint8_t Month;     // Месяц (1-12)
    uint8_t Date;      // Дата (1-31)
    uint8_t WeekDay;   // День недели (1-7)
    uint8_t Hours;     // Часы (0-23)
    uint8_t Minutes;   // Минуты (0-59)
    uint8_t Seconds;   // Секунды (0-59)
    uint8_t _1s256;    // Доли секунды (0-255)
} RtcDataTime;
```

### Дисплей (500)

| ID | Команда | Описание | Аргументы | Возврат |
|----|---------|----------|-----------|---------|
| 500 | `ssr_tft_fullscreen_repaint` | Полная перерисовка экрана | - | - |

## Справочник команд Long IO (Длинные команды)

Команды Long IO выполняются через запись в `long_request` с параметрами в `long_data[]`. Результат возвращается в `long_response` и `long_data[]`.

**Протокол выполнения:**
1. Мастер записывает команду в `long_request` (offset 37)
2. Мастер записывает количество аргументов в `long_data[0]` (offset 38)
3. Мастер записывает аргументы в `long_data[1..N]` (offset 39+)
4. Устройство обрабатывает команду через `handle_long_request()`
5. Устройство записывает код команды в `long_response` (offset 36)
6. Устройство записывает количество слов результата в `long_data[0]`
7. Устройство записывает результат в `long_data[1..M]`

### Системные команды Long IO (SystemLongRequestID)

| ID | Команда | Описание | Аргументы | Возврат |
|----|---------|----------|-----------|---------|
| 0 | `slr_none` | Нет операции | - | - |
| 1 | `slr_get_long_io_version` | Версия Long IO протокола | - | [1] версия (1) |
| 2 | `slr_get_long_io_length` | Размер буфера Long IO | - | [1] размер (66) |
| 3 | `slr_call_short_request_list_no_args_ret_1u16` | Batch вызов коротких команд без аргументов | [N] список ssr_* ID | [N] результаты |
| 4 | `slr_call_short_request_list_args_1u16_ret_1u16` | Batch вызов коротких команд с 1 аргументом | [2N] пары (ssr_ID, arg) | [2N] пары (response, result) |
| 5 | `slr_rs485_request` | RS485 запрос | [N] данные запроса | [M] данные ответа |
| 6 | `slr_get_int_sens_desc` | Описания внутренних датчиков | - | [N] SensorDescription[] |
| 7 | `slr_replace_masked_bits` | Маскированная запись битов (Long версия) | ReplaceMaskedBitsHeader + ReplaceMaskedBitsData[] | - |
| 8 | `slr_log_flash_status` | Статус Flash лога | - | [N] статус |
| 9 | `slr_log_flash_read` | Чтение Flash лога | [N] параметры | [M] данные лога |

**Структура SensorDescription:**
```c
typedef struct {
    uint8_t number;       // Номер датчика
    uint8_t invisible;    // Флаг видимости (0/1)
    uint8_t sensor_type;  // Тип датчика (см. sensor_type_*)
    uint8_t reserved;     // Резерв
} SensorDescription;
```

## Примеры короткого обмена (Short IO) через FC=16

Атомарная «маскированная запись» битов в CTRL_FLAGS (адрес 40013 / offset 12 в Holding) через окно DMEM:
- Предпосылка: DMEM_BASE известно; short_response по смещению 16, short_request — 17, short_data — с 18.
- Запрос (FC=16, старт = DMEM_BASE+17, количество = 1 + K):
  - Пишем short_request = 300 (ssr_replace_masked_bits)
  - Пишем в short_data: [dest_reg_addr=12, MASK=0x0001, VALUE=0x0001] — установка бита MEAS_ENABLE
- Ответ: стандартный FC=16 (эхо). После обработки слейв выставит short_response = 0 (OK).

**Пример 1: Получение версии Short IO протокола**
- Запрос (FC=16, DMEM_BASE+17, 2 слова):
  - short_request = 1 (ssr_get_short_io_version)
  - short_data[0] = 0 (количество аргументов)
- Ответ после обработки:
  - short_response = 1 (команда выполнена)
  - short_data[0] = 1 (количество слов результата)
  - short_data[1] = 1 (версия протокола)

**Пример 2: Получение RS485 адреса**
- Запрос (FC=16, DMEM_BASE+17, 2 слова):
  - short_request = 50 (ssr_get_rs485_id)
  - short_data[0] = 0 (количество аргументов)
- Ответ после обработки:
  - short_response = 50 (команда выполнена)
  - short_data[0] = 1 (количество слов результата)
  - short_data[1] = текущий_адрес (например, 1)

**Пример 3: Установка RS485 адреса**
- Запрос (FC=16, DMEM_BASE+17, 3 слова):
  - short_request = 51 (ssr_set_rs485_id)
  - short_data[0] = 1 (количество аргументов)
  - short_data[1] = 10 (новый адрес)
- Ответ после обработки:
  - short_response = 51 (команда выполнена)
  - short_data[0] = 1 (количество слов результата)
  - short_data[1] = 0 (код ошибки, 0=OK)

**Пример 4: Чтение времени RTC0**
- Запрос (FC=16, DMEM_BASE+17, 2 слова):
  - short_request = 400 (ssr_rtc0_read_date_time)
  - short_data[0] = 0 (количество аргументов)
- Ответ после обработки:
  - short_response = 400 (команда выполнена)
  - short_data[0] = 4 (количество слов результата)
  - short_data[1-4] = RtcDataTime структура (8 байт)

**Пример Long IO: Получение описаний внутренних датчиков**
- Запрос (FC=16, DMEM_BASE+37, 2 слова):
  - long_request = 6 (slr_get_int_sens_desc)
  - long_data[0] = 0 (количество аргументов)
- Ответ после обработки:
  - long_response = 6 (команда выполнена)
  - long_data[0] = N (количество слов результата)
  - long_data[1-N] = массив SensorDescription[7] (по 4 байта на датчик = 14 слов)
