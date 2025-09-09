# Modbus API

Полное описание протокольных параметров, окна памяти DeviceMemoryModbus и Short/Long IO команд.

Содержание:
- Параметры транспорта
- Пространство адресов и регистры (только DeviceMemoryModbus)
- Типы и единицы измерения
- Исключения и ошибки
- Тайминги RTU
- Идентификация устройства
- Профиль совместимости с Heater Control Modbus Master
- Окно памяти DeviceMemoryModbus (Holding)
- Справочник команд Short IO
- Справочник команд Long IO
- Примеры короткого и длинного обмена через окно DMEM

## Параметры транспорта

- Режим: Modbus RTU (Slave).
- Адрес ведомого (Slave ID): по умолчанию 1 (диапазон 1…247).
- UART параметры по умолчанию: 9600, 8N1 (настраивается).
- Основные используемые FC:
  - 03 Read Holding Registers — чтение окна DeviceMemoryModbus
  - 16 Write Multiple Registers — запись окна DeviceMemoryModbus и выполнение Short/Long IO
  - 43/14 Read Device Identification (опционально)
- CRC-16 (Modbus): полином 0xA001, начальное значение 0xFFFF, порядок байтов — LSB first.

USB-CDC (сервис/отладка): канал логирования и сервисных команд с ПК/приложения.

## Пространство адресов и регистры

Примечание:
- В данной прошивке не реализованы отдельные карты Discrete Inputs (1xxxx), Coils (0xxxx), Input Registers (3xxxx) и фиксированная карта Holding (4xxxx) с перечисленными регистрами.
- Все взаимодействие с устройством выполняется через окно памяти DeviceMemoryModbus, размещенное в области Holding регистров (см. раздел «Окно памяти DeviceMemoryModbus (Holding)»), а также через Short IO и Long IO команды.

## Типы и единицы измерения

- Температура: signed int16, шаг 0.01 °C, диапазон −327.68…+327.67 °C.
- Мощность/процент: uint16, шаг 0.1 % (0..1000).
- Напряжение: mV, uint16.
- Версия: два регистра (major.minor и patch) — используются в структуре/данных команд.
- Битовые поля статусов: см. STATUS_BITS в данных/структурах команд (если возвращаются соответствующими командами).

## Исключения и ошибки

Коды исключений Modbus:
- 01 Illegal Function
- 02 Illegal Data Address
- 03 Illegal Data Value
- 04 Slave Device Failure
- 06 Slave Device Busy
- 08 Memory Parity Error (используется по необходимости)

LAST_ERROR_CODE (уровня устройства, если возвращается командами):
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
- Время ответа: зависит от скорости UART и выполняемой команды.

## Идентификация устройства (опционально, 43/14)

- VendorName: "h-id"
- ProductCode: "heater-sensor"
- MajorMinorRevision: "MAJOR.MINOR.PATCH"
- Additional: серийный номер/датакод (если доступно)

## Профиль совместимости с Heater Control Modbus Master

Для корректной работы с dobord/h-id_heater_control_modbus_master_stm32f401ccu6 требуется:

1) Использование окна DeviceMemoryModbus:
- Мастер взаимодействует с устройством через фиксированное окно DMEM (см. ниже) по FC=03/16.

2) Управляющее слово и маскированные операции:
- Управляющие флаги доступны в области dev_ctl в окне DMEM.
- Для атомарного изменения битов используйте Short IO/Long IO команду replace_masked_bits (см. команды 300 и slr 7).

3) Short IO Service Area:
- Выполнение сервисных операций без гонок через short_request/short_data/short_response.

Примечания:
- Конкретные OPCODE и структуры синхронизированы с реализацией мастера.

## Окно памяти DeviceMemoryModbus (Holding)

Ниже описана логическая раскладка массива регистров «DeviceMemoryModbus», используемого для унифицированного доступа.

```c
typedef struct DeviceMemoryModbus {
    uint16_t reserved_relay_out[16];                              // offset = 0-15
    uint16_t short_response;                                      // offset = 16 (FIXED)
    uint16_t short_request;                                       // offset = 17
    uint16_t short_data[9];                                       // offset = 18-26 (hscc_short_io_length = 9)
    uint16_t int_sens_status[1];                                  // offset = 27 (length_of_ctl_reg_internal_sensors = 1)
    uint16_t int_sens_value[7];                                   // offset = 28-34 (max_num_of_internal_sensors = 7)
    DeviceControlRegister_SensorTemperature dev_ctl;              // offset = 35
    uint16_t long_response;                                       // offset = 36
    uint16_t long_request;                                        // offset = 37  
    uint16_t long_data[66];                                       // offset = 38-103 (hscc_long_io_length = 2 + MAX_BUFFER/2 = 66)
} DeviceMemoryModbus;
```

Константы структуры:
- `hscc_short_io_length = 9` — размер буфера коротких команд
- `hscc_long_io_length = 66` — размер буфера длинных команд (2 + MAX_BUFFER/2, MAX_BUFFER=128)
- `max_num_of_internal_sensors = 7` — количество внутренних датчиков (vref, vbat, tcpu, t1, t2, t3, t4)
- `length_of_ctl_reg_internal_sensors = 1` — размер статусного регистра датчиков ((7+15)/16)
- `hscc_short_io_version = 1` — версия протокола коротких команд
- `hscc_long_io_version = 1` — версия протокола длинных команд

Статистика реализации:
- Short IO команд: 23 из 50 определенных (46%)
- Long IO команд: 6 из 10 определенных (60%)
- Общий размер DeviceMemoryModbus: 104 слова (208 байт)

Внутренние датчики:
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

- [16] short_response — код ответа короткого обмена.
  - Тип: uint16.
  - Доступ: R (ведомое обновляет после обработки запроса).
  - Семантика: 0 = OK; !=0 — код ошибки.

- [17] short_request — код запроса короткого обмена (OPCODE).
  - Тип: uint16.
  - Доступ: W (мастер пишет OPCODE, затем параметры в short_data).

- [18..18+hscc_short_io_length-1] short_data — параметры/данные короткого обмена.
  - Тип: массив uint16.
  - Доступ: R/W.

- [18+hscc_short_io_length .. + length_of_ctl_reg_internal_sensors-1] int_sens_status[] — статус внутренних датчиков.
  - Тип: массив uint16; битовые флаги наличия/ошибок по датчикам.
  - Доступ: R.

- Далее int_sens_value[] — значения внутренних датчиков.
  - Тип: массив uint16; формат значений соответствует прибору (например, температура в 0.01 °C).
  - Доступ: R.

- dev_ctl — регистр(ы) управления устройством (DeviceControlRegister_SensorTemperature).
  - Тип: одно или несколько 16‑битных слов.
  - Доступ: R/W.

- long_response / long_request / long_data[] — порт обмена длинными сообщениями (аналог short_*).
  - long_response — R, код завершения операции.
  - long_request — W, OPCODE длинной операции.
  - long_data[] — R/W, буфер параметров/данных длиной hscc_long_io_length слов.

Примечания к окну DMEM:
- Смещение short_response фиксировано = 16 независимо от версии устройства.
- Общий размер DeviceMemoryModbus: 104 слова (208 байт) — 16 + 1 + 1 + 9 + 1 + 7 + 1 + 1 + 1 + 66 = 104.

## Справочник команд Short IO (Короткие команды)

Команды Short IO выполняются через запись в `short_request` с параметрами в `short_data[]`. Результат возвращается в `short_response` и `short_data[]`.

Протокол выполнения:
1. Мастер записывает команду в `short_request` (offset 17)
2. Мастер записывает количество аргументов в `short_data[0]` (offset 18)
3. Мастер записывает аргументы в `short_data[1..N]` (offset 19+)
4. Устройство обрабатывает команду через `handle_short_request()`
5. Устройство записывает код команды в `short_response` (offset 16)
6. Устройство записывает количество слов результата в `short_data[0]`
7. Устройство записывает результат в `short_data[1..M]`

### Системные команды Short IO (SystemShortRequestID)

Примечание: ✅ = реализовано, ⚪ = только определено в enum

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 0 | `ssr_none` | ✅ | Нет операции | - | - |
| 1 | `ssr_get_short_io_version` | ✅ | Версия Short IO протокола | - | [1] версия (1) |
| 2 | `ssr_get_short_io_length` | ✅ | Размер буфера Short IO | - | [1] размер (9) |
| 3 | `ssr_get_long_io_version` | ✅ | Версия Long IO протокола | - | [1] версия (1) |
| 4 | `ssr_get_long_io_length` | ✅ | Размер буфера Long IO | - | [1] размер (66) |
| 5 | `ssr_get_software_type` | ✅ | Тип программного обеспечения | - | [1] тип |
| 6 | `ssr_get_device_memory_modbus_length` | ✅ | Размер DeviceMemoryModbus | - | [1] размер в словах |

### RS485 и идентификация (50-86)

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 50 | `ssr_get_rs485_id` | ✅ | Получить RS485 адрес | - | [1] текущий адрес |
| 51 | `ssr_set_rs485_id` | ✅ | Установить RS485 адрес | [1] новый адрес | [1] код ошибки |
| 52 | `ssr_get_devid` | ✅ | Получить Device ID | - | [2] HAL_GetDEVID() |
| 53 | `ssr_get_revid` | ✅ | Получить Revision ID | - | [2] HAL_GetREVID() |
| 54 | `ssr_get_uid` | ✅ | Получить Unique ID | - | [6] HAL_GetUIDw0/1/2() |

### EEPROM операции (90-92)

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 90 | `ssr_eeprom_factory_defaults` | ✅ | Восстановить заводские настройки | - | [1] код результата |
| 91 | `ssr_eeprom_load_config` | ✅ | Загрузить конфигурацию | - | [1] код результата |
| 92 | `ssr_eeprom_save_config` | ✅ | Сохранить конфигурацию | - | [1] код результата |

### Адреса в DeviceMemoryModbus (100-141)

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 100 | `ssr_get_addr_long_io` | ✅ | Адрес long_response | - | [1] offset/2 |
| 101 | `ssr_get_addr_dev_ctl` | ✅ | Адрес dev_ctl | - | [1] offset/2 |
| 102 | `ssr_get_addr_common_ctl` | ⚪ | Адрес common_ctl | - | [1] offset/2 |
| 103 | `ssr_get_addr_heater_ctl` | ⚪ | Адрес heater_ctl | - | [1] offset/2 |
| 104 | `ssr_get_addr_heater_param` | ⚪ | Адрес heater_param | - | [1] offset/2 |
| 105 | `ssr_get_addr_heater_counter_q` | ⚪ | Адрес heater_counter_q | - | [1] offset/2 |
| 110 | `ssr_get_addr_int_sens_status` | ✅ | Адрес int_sens_status | - | [1] offset/2 |
| 111 | `ssr_get_addr_int_sens_value` | ✅ | Адрес int_sens_value | - | [1] offset/2 |
| 120 | `ssr_get_addr_sens_addr` | ⚪ | Адрес sens_addr | - | [1] offset/2 |
| 121 | `ssr_get_addr_sens_desc` | ⚪ | Адрес sens_desc | - | [1] offset/2 |
| 122 | `ssr_get_addr_sens_enable` | ⚪ | Адрес sens_enable | - | [1] offset/2 |
| 123 | `ssr_get_addr_sens_status` | ⚪ | Адрес sens_status | - | [1] offset/2 |
| 124 | `ssr_get_addr_sens_value` | ⚪ | Адрес sens_value | - | [1] offset/2 |
| 130 | `ssr_get_addr_relay_ctl` | ⚪ | Адрес relay_ctl | - | [1] offset/2 |
| 131 | `ssr_get_addr_relay_status` | ⚪ | Адрес relay_status | - | [1] offset/2 |
| 132 | `ssr_get_addr_relay_value` | ⚪ | Адрес relay_value | - | [1] offset/2 |
| 140 | `ssr_get_addr_display_output_control` | ⚪ | Адрес display_output_control | - | [1] offset/2 |
| 141 | `ssr_get_addr_display_variable` | ⚪ | Адрес display_variable | - | [1] offset/2 |

### Максимальные значения (200-206)

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 200 | `ssr_get_max_num_of_internal_sensors` | ✅ | Макс. кол-во внутренних датчиков | - | [1] значение (7) |
| 201 | `ssr_get_max_num_of_internal_relays` | ✅ | Макс. кол-во внутренних реле | - | [1] значение (8) |
| 202 | `ssr_get_max_num_of_sensors` | ⚪ | Макс. кол-во датчиков | - | [1] значение |
| 203 | `ssr_get_max_num_of_relays` | ⚪ | Макс. кол-во реле | - | [1] значение |
| 204 | `ssr_get_max_num_of_heater_ctl` | ⚪ | Макс. кол-во heater_ctl | - | [1] значение |
| 205 | `ssr_get_max_num_of_common_ctl` | ⚪ | Макс. кол-во common_ctl | - | [1] значение |
| 206 | `ssr_get_max_num_of_display_variable` | ⚪ | Макс. кол-во display_variable | - | [1] значение |

### Битовые операции (300)

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 300 | `ssr_replace_masked_bits` | ✅ | Маскированная запись битов | ReplaceMaskedBitsHeader + ReplaceMaskedBitsData[] | - |

Структура ReplaceMaskedBitsHeader:
```c
typedef struct {
    uint16_t reg_addr_start;  // Начальный адрес в DeviceMemoryModbus
    uint16_t num;             // Количество регистров для обработки
} ReplaceMaskedBitsHeader;
```

Структура ReplaceMaskedBitsData:
```c
typedef struct {
    uint16_t mask;   // Битовая маска
    uint16_t data;   // Новые биты
} ReplaceMaskedBitsData;
```

### RTC операции (400-434)

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 400 | `ssr_rtc0_read_date_time` | ✅ | Чтение даты/времени RTC0 | - | [4] RtcDataTime |
| 401 | `ssr_rtc0_write_date_time` | ✅ | Запись даты/времени RTC0 | [4] RtcDataTime | - |
| 420 | `ssr_rtc1_read_date_time` | ⚪ | Чтение даты/времени RTC1 | - | [4] RtcDataTime |
| 421 | `ssr_rtc1_write_date_time` | ⚪ | Запись даты/времени RTC1 | [4] RtcDataTime | - |
| 422 | `ssr_rtc1_read_calibr` | ⚪ | Чтение калибровки RTC1 | - | [1] значение |
| 423 | `ssr_rtc1_write_calibr` | ⚪ | Запись калибровки RTC1 | [1] значение | - |
| 424 | `ssr_rtc1_read_regs` | ⚪ | Чтение регистров RTC1 | - | [N] регистры |
| 425 | `ssr_rtc1_write_regs` | ⚪ | Запись регистров RTC1 | [N] регистры | - |
| 426 | `ssr_rtc1_copy_to_rtc0` | ⚪ | Копирование RTC1 → RTC0 | - | - |

Структура RtcDataTime:
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

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 500 | `ssr_tft_fullscreen_repaint` | ⚪ | Полная перерисовка экрана | - | - |

## Справочник команд Long IO (Длинные команды)

Команды Long IO выполняются через запись в `long_request` с параметрами в `long_data[]`. Результат возвращается в `long_response` и `long_data[]`.

Протокол выполнения:
1. Мастер записывает команду в `long_request` (offset 37)
2. Мастер записывает количество аргументов в `long_data[0]` (offset 38)
3. Мастер записывает аргументы в `long_data[1..N]` (offset 39+)
4. Устройство обрабатывает команду через `handle_long_request()`
5. Устройство записывает код команды в `long_response` (offset 36)
6. Устройство записывает количество слов результата в `long_data[0]`
7. Устройство записывает результат в `long_data[1..M]`

### Системные команды Long IO (SystemLongRequestID)

Примечание: ✅ = реализовано, ⚪ = только определено в enum

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 0 | `slr_none` | ✅ | Нет операции | - | - |
| 1 | `slr_get_long_io_version` | ✅ | Версия Long IO протокола | - | [1] версия (1) |
| 2 | `slr_get_long_io_length` | ✅ | Размер буфера Long IO | - | [1] размер (66) |
| 3 | `slr_call_short_request_list_no_args_ret_1u16` | ✅ | Batch вызов коротких команд без аргументов | [N] список ssr_* ID | [N] результаты |
| 4 | `slr_call_short_request_list_args_1u16_ret_1u16` | ✅ | Batch вызов коротких команд с 1 аргументом | [2N] пары (ssr_ID, arg) | [2N] пары (response, result) |
| 5 | `slr_rs485_request` | ⚪ | RS485 запрос | [N] данные запроса | [M] данные ответа |
| 6 | `slr_get_int_sens_desc` | ✅ | Описания внутренних датчиков | - | [N] SensorDescription[] |
| 7 | `slr_replace_masked_bits` | ✅ | Маскированная запись битов (Long версия) | ReplaceMaskedBitsHeader + ReplaceMaskedBitsData[] | - |
| 8 | `slr_log_flash_status` | ⚪ | Статус Flash лога | - | [N] статус |
| 9 | `slr_log_flash_read` | ⚪ | Чтение Flash лога | [N] параметры | [M] данные лога |

Структура SensorDescription:
```c
typedef struct {
    uint8_t number;       // Номер датчика
    uint8_t invisible;    // Флаг видимости (0/1)
    uint8_t sensor_type;  // Тип датчика (см. sensor_type_*)
    uint8_t reserved;     // Резерв
} SensorDescription;
```

## Примеры короткого обмена (Short IO) через FC=16

Атомарная «маскированная запись» битов в произвольный диапазон DeviceMemoryModbus:
- Предпосылка: DMEM_BASE известно; short_response по смещению 16, short_request — 17, short_data — с 18.
- Запрос (FC=16, старт = DMEM_BASE+17, количество = 1 + K):
  - Пишем short_request = 300 (ssr_replace_masked_bits)
  - В short_data: сначала header и данные, например:
    - header: reg_addr_start = 35 (адрес dev_ctl), num = 1
    - data[0]: MASK=0x0001, VALUE=0x0001 — установка бита Enable
- Ответ: стандартный FC=16 (эхо). После обработки слейв выставит short_response = 300 (код выполненной команды), а short_data[0] = 0.

Пример 1: Получение версии Short IO протокола
- Запрос (FC=16, DMEM_BASE+17, 2 слова):
  - short_request = 1 (ssr_get_short_io_version)
  - short_data[0] = 0 (количество аргументов)
- Ответ:
  - short_response = 1
  - short_data[0] = 1
  - short_data[1] = 1 (версия)

Пример 2: Получение RS485 адреса
- Запрос (FC=16, DMEM_BASE+17, 2 слова):
  - short_request = 50 (ssr_get_rs485_id)
  - short_data[0] = 0
- Ответ:
  - short_response = 50
  - short_data[0] = 1
  - short_data[1] = текущий_адрес

Пример 3: Установка RS485 адреса
- Запрос (FC=16, DMEM_BASE+17, 3 слова):
  - short_request = 51 (ssr_set_rs485_id)
  - short_data[0] = 1
  - short_data[1] = 10 (новый адрес)
- Ответ:
  - short_response = 51
  - short_data[0] = 1
  - short_data[1] = 0 (код ошибки, 0=OK)

Пример 4: Чтение времени RTC0
- Запрос (FC=16, DMEM_BASE+17, 2 слова):
  - short_request = 400 (ssr_rtc0_read_date_time)
  - short_data[0] = 0
- Ответ:
  - short_response = 400
  - short_data[0] = 4
  - short_data[1-4] = RtcDataTime (8 байт)

Пример Long IO: Получение описаний внутренних датчиков
- Запрос (FC=16, DMEM_BASE+37, 2 слова):
  - long_request = 6 (slr_get_int_sens_desc)
  - long_data[0] = 0
- Ответ:
  - long_response = 6
  - long_data[0] = N (количество слов результата)
  - long_data[1-N] = массив SensorDescription[7] (по 4 байта на датчик = 14 слов)

### Системные команды Long IO (SystemLongRequestID)

Примечание: ✅ = реализовано, ⚪ = только определено в enum

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 0 | `slr_none` | ✅ | Нет операции | - | - |
| 1 | `slr_get_long_io_version` | ✅ | Версия Long IO протокола | - | [1] версия (1) |
| 2 | `slr_get_long_io_length` | ✅ | Размер буфера Long IO | - | [1] размер (66) |
| 3 | `slr_call_short_request_list_no_args_ret_1u16` | ✅ | Batch вызов коротких команд без аргументов | [N] список ssr_* ID | [N] результаты |
| 4 | `slr_call_short_request_list_args_1u16_ret_1u16` | ✅ | Batch вызов коротких команд с 1 аргументом | [2N] пары (ssr_ID, arg) | [2N] пары (response, result) |
| 5 | `slr_rs485_request` | ⚪ | RS485 запрос | [N] данные запроса | [M] данные ответа |
| 6 | `slr_get_int_sens_desc` | ✅ | Описания внутренних датчиков | - | [N] SensorDescription[] |
| 7 | `slr_replace_masked_bits` | ✅ | Маскированная запись битов (Long версия) | ReplaceMaskedBitsHeader + ReplaceMaskedBitsData[] | - |
| 8 | `slr_log_flash_status` | ⚪ | Статус Flash лога | - | [N] статус |
| 9 | `slr_log_flash_read` | ⚪ | Чтение Flash лога | [N] параметры | [M] данные лога |

Структура SensorDescription:
```c
typedef struct {
    uint8_t number;       // Номер датчика
    uint8_t invisible;    // Флаг видимости (0/1)
    uint8_t sensor_type;  // Тип датчика (см. sensor_type_*)
    uint8_t reserved;     // Резерв
} SensorDescription;
```

## Примеры короткого обмена (Short IO) через FC=16

Атомарная «маскированная запись» битов в произвольный диапазон DeviceMemoryModbus:
- Предпосылка: DMEM_BASE известно; short_response по смещению 16, short_request — 17, short_data — с 18.
- Запрос (FC=16, старт = DMEM_BASE+17, количество = 1 + K):
  - Пишем short_request = 300 (ssr_replace_masked_bits)
  - В short_data: сначала header и данные, например:
    - header: reg_addr_start = 35 (адрес dev_ctl), num = 1
    - data[0]: MASK=0x0001, VALUE=0x0001 — установка бита Enable
- Ответ: стандартный FC=16 (эхо). После обработки слейв выставит short_response = 300 (код выполненной команды), а short_data[0] = 0.

Пример 1: Получение версии Short IO протокола
- Запрос (FC=16, DMEM_BASE+17, 2 слова):
  - short_request = 1 (ssr_get_short_io_version)
  - short_data[0] = 0 (количество аргументов)
- Ответ:
  - short_response = 1
  - short_data[0] = 1
  - short_data[1] = 1 (версия)

Пример 2: Получение RS485 адреса
- Запрос (FC=16, DMEM_BASE+17, 2 слова):
  - short_request = 50 (ssr_get_rs485_id)
  - short_data[0] = 0
- Ответ:
  - short_response = 50
  - short_data[0] = 1
  - short_data[1] = текущий_адрес

Пример 3: Установка RS485 адреса
- Запрос (FC=16, DMEM_BASE+17, 3 слова):
  - short_request = 51 (ssr_set_rs485_id)
  - short_data[0] = 1
  - short_data[1] = 10 (новый адрес)
- Ответ:
  - short_response = 51
  - short_data[0] = 1
  - short_data[1] = 0 (код ошибки, 0=OK)

Пример 4: Чтение времени RTC0
- Запрос (FC=16, DMEM_BASE+17, 2 слова):
  - short_request = 400 (ssr_rtc0_read_date_time)
  - short_data[0] = 0
- Ответ:
  - short_response = 400
  - short_data[0] = 4
  - short_data[1-4] = RtcDataTime (8 байт)

Пример Long IO: Получение описаний внутренних датчиков
- Запрос (FC=16, DMEM_BASE+37, 2 слова):
  - long_request = 6 (slr_get_int_sens_desc)
  - long_data[0] = 0
- Ответ:
  - long_response = 6
  - long_data[0] = N (количество слов результата)
  - long_data[1-N] = массив SensorDescription[7] (по 4 байта на датчик = 14 слов)

### Системные команды Long IO (SystemLongRequestID)

Примечание: ✅ = реализовано, ⚪ = только определено в enum

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 0 | `slr_none` | ✅ | Нет операции | - | - |
| 1 | `slr_get_long_io_version` | ✅ | Версия Long IO протокола | - | [1] версия (1) |
| 2 | `slr_get_long_io_length` | ✅ | Размер буфера Long IO | - | [1] размер (66) |
| 3 | `slr_call_short_request_list_no_args_ret_1u16` | ✅ | Batch вызов коротких команд без аргументов | [N] список ssr_* ID | [N] результаты |
| 4 | `slr_call_short_request_list_args_1u16_ret_1u16` | ✅ | Batch вызов коротких команд с 1 аргументом | [2N] пары (ssr_ID, arg) | [2N] пары (response, result) |
| 5 | `slr_rs485_request` | ⚪ | RS485 запрос | [N] данные запроса | [M] данные ответа |
| 6 | `slr_get_int_sens_desc` | ✅ | Описания внутренних датчиков | - | [N] SensorDescription[] |
| 7 | `slr_replace_masked_bits` | ✅ | Маскированная запись битов (Long версия) | ReplaceMaskedBitsHeader + ReplaceMaskedBitsData[] | - |
| 8 | `slr_log_flash_status` | ⚪ | Статус Flash лога | - | [N] статус |
| 9 | `slr_log_flash_read` | ⚪ | Чтение Flash лога | [N] параметры | [M] данные лога |

Структура SensorDescription:
```c
typedef struct {
    uint8_t number;       // Номер датчика
    uint8_t invisible;    // Флаг видимости (0/1)
    uint8_t sensor_type;  // Тип датчика (см. sensor_type_*)
    uint8_t reserved;     // Резерв
} SensorDescription;
```

## Примеры короткого обмена (Short IO) через FC=16

Атомарная «маскированная запись» битов в произвольный диапазон DeviceMemoryModbus:
- Предпосылка: DMEM_BASE известно; short_response по смещению 16, short_request — 17, short_data — с 18.
- Запрос (FC=16, старт = DMEM_BASE+17, количество = 1 + K):
  - Пишем short_request = 300 (ssr_replace_masked_bits)
  - В short_data: сначала header и данные, например:
    - header: reg_addr_start = 35 (адрес dev_ctl), num = 1
    - data[0]: MASK=0x0001, VALUE=0x0001 — установка бита Enable
- Ответ: стандартный FC=16 (эхо). После обработки слейв выставит short_response = 300 (код выполненной команды), а short_data[0] = 0.

Пример 1: Получение версии Short IO протокола
- Запрос (FC=16, DMEM_BASE+17, 2 слова):
  - short_request = 1 (ssr_get_short_io_version)
  - short_data[0] = 0 (количество аргументов)
- Ответ:
  - short_response = 1
  - short_data[0] = 1
  - short_data[1] = 1 (версия)

Пример 2: Получение RS485 адреса
- Запрос (FC=16, DMEM_BASE+17, 2 слова):
  - short_request = 50 (ssr_get_rs485_id)
  - short_data[0] = 0
- Ответ:
  - short_response = 50
  - short_data[0] = 1
  - short_data[1] = текущий_адрес

Пример 3: Установка RS485 адреса
- Запрос (FC=16, DMEM_BASE+17, 3 слова):
  - short_request = 51 (ssr_set_rs485_id)
  - short_data[0] = 1
  - short_data[1] = 10 (новый адрес)
- Ответ:
  - short_response = 51
  - short_data[0] = 1
  - short_data[1] = 0 (код ошибки, 0=OK)

Пример 4: Чтение времени RTC0
- Запрос (FC=16, DMEM_BASE+17, 2 слова):
  - short_request = 400 (ssr_rtc0_read_date_time)
  - short_data[0] = 0
- Ответ:
  - short_response = 400
  - short_data[0] = 4
  - short_data[1-4] = RtcDataTime (8 байт)

Пример Long IO: Получение описаний внутренних датчиков
- Запрос (FC=16, DMEM_BASE+37, 2 слова):
  - long_request = 6 (slr_get_int_sens_desc)
  - long_data[0] = 0
- Ответ:
  - long_response = 6
  - long_data[0] = N (количество слов результата)
  - long_data[1-N] = массив SensorDescription[7] (по 4 байта на датчик = 14 слов)

### Системные команды Long IO (SystemLongRequestID)

Примечание: ✅ = реализовано, ⚪ = только определено в enum

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 0 | `slr_none` | ✅ | Нет операции | - | - |
| 1 | `slr_get_long_io_version` | ✅ | Версия Long IO протокола | - | [1] версия (1) |
| 2 | `slr_get_long_io_length` | ✅ | Размер буфера Long IO | - | [1] размер (66) |
| 3 | `slr_call_short_request_list_no_args_ret_1u16` | ✅ | Batch вызов коротких команд без аргументов | [N] список ssr_* ID | [N] результаты |
| 4 | `slr_call_short_request_list_args_1u16_ret_1u16` | ✅ | Batch вызов коротких команд с 1 аргументом | [2N] пары (ssr_ID, arg) | [2N] пары (response, result) |
| 5 | `slr_rs485_request` | ⚪ | RS485 запрос | [N] данные запроса | [M] данные ответа |
| 6 | `slr_get_int_sens_desc` | ✅ | Описания внутренних датчиков | - | [N] SensorDescription[] |
| 7 | `slr_replace_masked_bits` | ✅ | Маскированная запись битов (Long версия) | ReplaceMaskedBitsHeader + ReplaceMaskedBitsData[] | - |
| 8 | `slr_log_flash_status` | ⚪ | Статус Flash лога | - | [N] статус |
| 9 | `slr_log_flash_read` | ⚪ | Чтение Flash лога | [N] параметры | [M] данные лога |

Структура SensorDescription:
```c
typedef struct {
    uint8_t number;       // Номер датчика
    uint8_t invisible;    // Флаг видимости (0/1)
    uint8_t sensor_type;  // Тип датчика (см. sensor_type_*)
    uint8_t reserved;     // Резерв
} SensorDescription;
```

## Примеры короткого обмена (Short IO) через FC=16

Атомарная «маскированная запись» битов в произвольный диапазон DeviceMemoryModbus:
- Предпосылка: DMEM_BASE известно; short_response по смещению 16, short_request — 17, short_data — с 18.
- Запрос (FC=16, старт = DMEM_BASE+17, количество = 1 + K):
  - Пишем short_request = 300 (ssr_replace_masked_bits)
  - В short_data: сначала header и данные, например:
    - header: reg_addr_start = 35 (адрес dev_ctl), num = 1
    - data[0]: MASK=0x0001, VALUE=0x0001 — установка бита Enable
- Ответ: стандартный FC=16 (эхо). После обработки слейв выставит short_response = 300 (код выполненной команды), а short_data[0] = 0.

Пример 1: Получение версии Short IO протокола
- Запрос (FC=16, DMEM_BASE+17, 2 слова):
  - short_request = 1 (ssr_get_short_io_version)
  - short_data[0] = 0 (количество аргументов)
- Ответ:
  - short_response = 1
  - short_data[0] = 1
  - short_data[1] = 1 (версия)

Пример 2: Получение RS485 адреса
- Запрос (FC=16, DMEM_BASE+17, 2 слова):
  - short_request = 50 (ssr_get_rs485_id)
  - short_data[0] = 0
- Ответ:
  - short_response = 50
  - short_data[0] = 1
  - short_data[1] = текущий_адрес

Пример 3: Установка RS485 адреса
- Запрос (FC=16, DMEM_BASE+17, 3 слова):
  - short_request = 51 (ssr_set_rs485_id)
  - short_data[0] = 1
  - short_data[1] = 10 (новый адрес)
- Ответ:
  - short_response = 51
  - short_data[0] = 1
  - short_data[1] = 0 (код ошибки, 0=OK)

Пример 4: Чтение времени RTC0
- Запрос (FC=16, DMEM_BASE+17, 2 слова):
  - short_request = 400 (ssr_rtc0_read_date_time)
  - short_data[0] = 0
- Ответ:
  - short_response = 400
  - short_data[0] = 4
  - short_data[1-4] = RtcDataTime (8 байт)

Пример Long IO: Получение описаний внутренних датчиков
- Запрос (FC=16, DMEM_BASE+37, 2 слова):
  - long_request = 6 (slr_get_int_sens_desc)
  - long_data[0] = 0
- Ответ:
  - long_response = 6
  - long_data[0] = N (количество слов результата)
  - long_data[1-N] = массив SensorDescription[7] (по 4 байта на датчик = 14 слов)

## Commit message: Docs: remove non-implemented Modbus register maps; focus on DeviceMemoryModbus (DMEM)