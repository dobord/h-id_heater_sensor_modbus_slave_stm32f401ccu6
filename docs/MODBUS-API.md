# Modbus API

Полное описание протокольных параметров и унифицированного окна памяти DeviceMemoryModbus (DMEM), а также справочники Short/Long IO команд. Документ очищен от дубликатов, разделы систематизированы и пронумерованы.

Содержание:
1. [Параметры транспорта](#1-параметры-транспорта)
2. [Пространство адресов и регистры](#2-пространство-адресов-и-регистры)
3. [Типы и единицы измерения](#3-типы-и-единицы-измерения)
4. [Исключения и ошибки](#4-исключения-и-ошибки)
5. [Тайминги RTU](#5-тайминги-rtu)
6. [Идентификация устройства](#6-идентификация-устройства-опционально-4314)
7. [Профиль совместимости с Heater Control Modbus Master](#7-профиль-совместимости-с-heater-control-modbus-master)
8. [Окно памяти DeviceMemoryModbus (Holding)](#8-окно-памяти-devicememorymodbus-holding)
   - 8.1 [Структура и константы](#81-структура-и-константы)
   - 8.2 [Статистика реализации](#82-статистика-реализации)
   - 8.3 [Внутренние датчики](#83-внутренние-датчики)
   - 8.4 [Поля окна и протокол обмена](#84-поля-окна-и-протокол-обмена)
9. [Short IO (короткие команды)](#9-short-io-короткие-команды)
   - 9.1 [Протокол выполнения](#91-протокол-выполнения)
   - 9.2 [Системные команды](#92-системные-команды-systemshortrequestid)
   - 9.3 [RS485 и идентификация](#93-rs485-и-идентификация-50-54)
   - 9.4 [EEPROM операции](#94-eeprom-операции-90-92)
   - 9.5 [Адреса в DeviceMemoryModbus](#95-адреса-в-devicememorymodbus-100-141)
   - 9.6 [Максимальные значения](#96-максимальные-значения-200-206)
   - 9.7 [Битовые операции](#97-битовые-операции-300)
   - 9.8 [RTC операции](#98-rtc-операции-400-426)
   - 9.9 [Дисплей](#99-дисплей-500)
10. [Long IO (длинные команды)](#10-long-io-длинные-команды)
    - 10.1 [Протокол выполнения](#101-протокол-выполнения)
    - 10.2 [Системные команды](#102-системные-команды-systemlongrequestid)
11. [Примеры обмена](#11-примеры-обмена)
    - 11.1 [Short IO (FC=16)](#111-short-io-fc16)
    - 11.2 [Long IO (FC=16)](#112-long-io-fc16)

---

## 1. Параметры транспорта

- Режим: Modbus RTU (Slave)
- Адрес ведомого (Slave ID): по умолчанию 1 (диапазон 1…247)
- UART по умолчанию: 9600, 8N1 (настраивается)
- Основные FC:
  - 03 — Read Holding Registers (чтение окна DeviceMemoryModbus)
  - 16 — Write Multiple Registers (запись окна DeviceMemoryModbus и запуск Short/Long IO)
  - 43/14 — Read Device Identification (опционально)
- CRC-16 (Modbus): полином 0xA001, начальное значение 0xFFFF, порядок байтов LSB first
- USB-CDC (сервис/отладка): канал логирования и сервисных команд

## 2. Пространство адресов и регистры

- Специальные карты 0xxxx/1xxxx/3xxxx не используются.
- Все взаимодействие ведётся через единое окно DeviceMemoryModbus (DMEM) в области Holding регистров.
- Чтение — FC=03; запись и команды — FC=16.

## 3. Типы и единицы измерения

- Температура: int16, шаг 0.01 °C, диапазон −327.68…+327.67 °C
- Мощность/процент: uint16, шаг 0.1% (0…1000)
- Напряжение: mV, uint16
- Версия: два регистра (major.minor и patch) при необходимости
- Битовые поля статусов: см. описания команд/структур

## 4. Исключения и ошибки

Исключения Modbus:
- 01 Illegal Function
- 02 Illegal Data Address
- 03 Illegal Data Value
- 04 Slave Device Failure
- 06 Slave Device Busy
- 08 Memory Parity Error

Коды уровня устройства (если команда их возвращает): LAST_ERROR_CODE
- 0: OK
- 1: SENSOR_DISCONNECT
- 2: SENSOR_SHORT
- 3: ADC_OOR
- 4: OVER_TEMP
- 5: MODBUS_CRC
- 6: MODBUS_FORMAT
- 7: FLASH_CRC_FAIL
- 8+: зарезервировано

## 5. Тай��инги RTU

- Межсимвольная пауза: t1.5
- Межкадровая пауза: t3.5 (обязательна)
- Время ответа: зависит от скорости UART и команды

## 6. Идентификация устройства (опционально, 43/14)

- VendorName: "h-id"
- ProductCode: "heater-sensor"
- MajorMinorRevision: "MAJOR.MINOR.PATCH"
- Additional: серийный номер/датакод (если доступно)

## 7. Профиль совместимости с Heater Control Modbus Master

1) Окно DeviceMemoryModbus  
- Мастер работает через фиксированное окно DMEM по FC=03/16.

2) Управляющее слово и атомарные операции  
- Управляющие флаги в области dev_ctl.
- Для атомарной модификации битов применяйте replace_masked_bits (Short: 300; Long: 7).

3) Short IO Service Area  
- Сервисные операции через short_request/short_data/short_response без гонок.

Примечание: OPCODE и структуры синхронизированы с мастером dobord/h-id_heater_control_modbus_master_stm32f401ccu6.

## 8. Окно памяти DeviceMemoryModbus (Holding)

### 8.1 Структура и константы

```c
typedef struct DeviceMemoryModbus {
    uint16_t reserved_relay_out[16];                              // offset = 0..15
    uint16_t short_response;                                      // offset = 16 (FIXED)
    uint16_t short_request;                                       // offset = 17
    uint16_t short_data[9];                                       // offset = 18..26  (hscc_short_io_length = 9)
    uint16_t int_sens_status[1];                                  // offset = 27      (length_of_ctl_reg_internal_sensors = 1)
    uint16_t int_sens_value[7];                                   // offset = 28..34  (max_num_of_internal_sensors = 7)
    DeviceControlRegister_SensorTemperature dev_ctl;              // offset = 35
    uint16_t long_response;                                       // offset = 36
    uint16_t long_request;                                        // offset = 37
    uint16_t long_data[66];                                       // offset = 38..103 (hscc_long_io_length = 66)
} DeviceMemoryModbus;
```

Константы:
- hscc_short_io_length = 9
- hscc_long_io_length = 66  (2 + MAX_BUFFER/2, при MAX_BUFFER = 128)
- max_num_of_internal_sensors = 7  (vref, vbat, tcpu, t1, t2, t3, t4)
- length_of_ctl_reg_internal_sensors = 1  ((7+15)/16)
- hscc_short_io_version = 1
- hscc_long_io_version = 1

Общий размер DMEM: 104 слова (208 байт). Раскладка: 16 + 1 + 1 + 9 + 1 + 7 + 1 + 1 + 1 + 66 = 104.

### 8.2 Статистика реализации

- Short IO команд: 23 из 50 (46%)
- Long IO команд: 6 из 10 (60%)

### 8.3 Внутренние датчики

- isid_vref (0): Опорное напряжение [mV]
- isid_vbat (1): Напряжение батареи RTC [mV]
- isid_tcpu (2): Температура CPU [0.01 °C]
- isid_t1 (3): Датчик 1 [0.01 °C]
- isid_t2 (4): Датчик 2 [0.01 °C]
- isid_t3 (5): Датчик 3 [0.01 °C]
- isid_t4 (6): Датчик 4 [0.01 °C]

### 8.4 Поля окна и протокол обмена

- [0..15] reserved_relay_out[] — R/W, вендорские слова состояний/реле
- [16] short_response — R, код подтверждения выполнения Short IO:
  - при успехе: echo OPCODE (тот же ID, что был в short_request)
  - при ошибке: может возвращаться Modbus-исключение на FC=16 или особые коды согласно команде
- [17] short_request — W, OPCODE Short IO
- [18..26] short_data[] — R/W, буфер аргументов/результата Short IO
- [27] int_sens_status[] — R, битовые статусы внутренних датчиков
- [28..34] int_sens_value[] — R, значения внутренних датчиков
- [35] dev_ctl — R/W, регистр(ы) управления устройством (DeviceControlRegister_SensorTemperature)
- [36] long_response — R, echo OPCODE Long IO при успехе
- [37] long_request — W, OPCODE Long IO
- [38..103] long_data[] — R/W, буфер аргументов/результата Long IO

Протокол Short IO:
1) Мастер пишет short_request (offset 17)  
2) Мастер пишет short_data[0] = число аргументов  
3) Мастер пишет short_data[1..N] — аргументы  
4) Устройство обрабатывает  
5) Устройство ставит short_response = OPCODE  
6) Устройство пишет short_data[0] = число слов результата  
7) Устройство пишет short_data[1..M] — результат

Протокол Long IO — аналогично через long_request/long_data/long_response.

---

## 9. Short IO (короткие команды)

### 9.1 Протокол выполнения

См. п. 8.4. Вызов осуществляется одной транзакцией FC=16 (в область 17+).

### 9.2 Системные команды (SystemShortRequestID)

Примечание: ✅ = реализовано, ⚪ = только определено в enum

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 0 | ssr_none | ✅ | Нет операции | - | - |
| 1 | ssr_get_short_io_version | ✅ | Версия Short IO | - | [1] версия (1) |
| 2 | ssr_get_short_io_length | ✅ | Длина буфера Short IO | - | [1] длина (9) |
| 3 | ssr_get_long_io_version | ✅ | Версия Long IO | - | [1] версия (1) |
| 4 | ssr_get_long_io_length | ✅ | Длина буфера Long IO | - | [1] длина (66) |
| 5 | ssr_get_software_type | ✅ | Тип ПО | - | [1] тип |
| 6 | ssr_get_device_memory_modbus_length | ✅ | Размер DMEM | - | [1] слова |

### 9.3 RS485 и идентификация (50-54)

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 50 | ssr_get_rs485_id | ✅ | Прочитать RS485 адрес | - | [1] адрес |
| 51 | ssr_set_rs485_id | ✅ | Установить RS485 адрес | [1] адрес | [1] код ошибки |
| 52 | ssr_get_devid | ✅ | HAL_GetDEVID() | - | [2] DEVID |
| 53 | ssr_get_revid | ✅ | HAL_GetREVID() | - | [2] REVID |
| 54 | ssr_get_uid | ✅ | HAL_GetUIDw0/1/2() | - | [6] UID |

### 9.4 EEPROM операции (90-92)

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 90 | ssr_eeprom_factory_defaults | ✅ | Сброс к заводским | - | [1] код результата |
| 91 | ssr_eeprom_load_config | ✅ | Загрузка конфигурации | - | [1] код результата |
| 92 | ssr_eeprom_save_config | ✅ | Сохранение конфигурации | - | [1] код результата |

### 9.5 Адреса в DeviceMemoryModbus (100-141)

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 100 | ssr_get_addr_long_io | ✅ | Адрес long_response | - | [1] offset (в словах) |
| 101 | ssr_get_addr_dev_ctl | ✅ | Адрес dev_ctl | - | [1] offset (в словах) |
| 102 | ssr_get_addr_common_ctl | ⚪ | Адрес common_ctl | - | [1] offset |
| 103 | ssr_get_addr_heater_ctl | ⚪ | Адрес heater_ctl | - | [1] offset |
| 104 | ssr_get_addr_heater_param | ⚪ | Адрес heater_param | - | [1] offset |
| 105 | ssr_get_addr_heater_counter_q | ⚪ | Адрес heater_counter_q | - | [1] offset |
| 110 | ssr_get_addr_int_sens_status | ✅ | Адрес int_sens_status | - | [1] offset |
| 111 | ssr_get_addr_int_sens_value | ✅ | Адрес int_sens_value | - | [1] offset |
| 120 | ssr_get_addr_sens_addr | ⚪ | Адрес sens_addr | - | [1] offset |
| 121 | ssr_get_addr_sens_desc | ⚪ | Адрес sens_desc | - | [1] offset |
| 122 | ssr_get_addr_sens_enable | ⚪ | Адрес sens_enable | - | [1] offset |
| 123 | ssr_get_addr_sens_status | ⚪ | Адрес sens_status | - | [1] offset |
| 124 | ssr_get_addr_sens_value | ⚪ | Адрес sens_value | - | [1] offset |
| 130 | ssr_get_addr_relay_ctl | ⚪ | Адрес relay_ctl | - | [1] offset |
| 131 | ssr_get_addr_relay_status | ⚪ | Адрес relay_status | - | [1] offset |
| 132 | ssr_get_addr_relay_value | ⚪ | Адрес relay_value | - | [1] offset |
| 140 | ssr_get_addr_display_output_control | ⚪ | Адрес display_output_control | - | [1] offset |
| 141 | ssr_get_addr_display_variable | ⚪ | Адрес display_variable | - | [1] offset |

### 9.6 Максимальные значения (200-206)

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 200 | ssr_get_max_num_of_internal_sensors | ✅ | Макс. число внутренних датчиков | - | [1] 7 |
| 201 | ssr_get_max_num_of_internal_relays | ✅ | Макс. число внутренних реле | - | [1] 8 |
| 202 | ssr_get_max_num_of_sensors | ⚪ | Макс. число датчиков | - | [1] значение |
| 203 | ssr_get_max_num_of_relays | ⚪ | Макс. число реле | - | [1] значение |
| 204 | ssr_get_max_num_of_heater_ctl | ⚪ | Макс. число heater_ctl | - | [1] значение |
| 205 | ssr_get_max_num_of_common_ctl | ⚪ | Макс. число common_ctl | - | [1] значение |
| 206 | ssr_get_max_num_of_display_variable | ⚪ | Макс. число display_variable | - | [1] значение |

### 9.7 Битовые операции (300)

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 300 | ssr_replace_masked_bits | ✅ | Маскированная запись битов | ReplaceMaskedBitsHeader + ReplaceMaskedBitsData[] | - |

Структуры:
```c
typedef struct {
    uint16_t reg_addr_start;  // начальный адрес в DMEM (в словах)
    uint16_t num;             // количество регистров
} ReplaceMaskedBitsHeader;

typedef struct {
    uint16_t mask;  // битовая маска
    uint16_t data;  // новые биты
} ReplaceMaskedBitsData;
```

### 9.8 RTC операции (400-426)

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 400 | ssr_rtc0_read_date_time | ✅ | Чтение даты/времени RTC0 | - | [4] RtcDataTime |
| 401 | ssr_rtc0_write_date_time | ✅ | Запись даты/времени RTC0 | [4] RtcDataTime | - |
| 420 | ssr_rtc1_read_date_time | ⚪ | Чтение даты/времени RTC1 | - | [4] RtcDataTime |
| 421 | ssr_rtc1_write_date_time | ⚪ | Запись даты/времени RTC1 | [4] RtcDataTime | - |
| 422 | ssr_rtc1_read_calibr | ⚪ | Чтение калибровки RTC1 | - | [1] значение |
| 423 | ssr_rtc1_write_calibr | ⚪ | Запись калибровки RTC1 | [1] значение | - |
| 424 | ssr_rtc1_read_regs | ⚪ | Чтение регистров RTC1 | - | [N] регистры |
| 425 | ssr_rtc1_write_regs | ⚪ | Запись регистров RTC1 | [N] регистры | - |
| 426 | ssr_rtc1_copy_to_rtc0 | ⚪ | Копирование RTC1 → RTC0 | - | - |

Структура времени:
```c
typedef struct {
    uint8_t Year;      // 0-99
    uint8_t Month;     // 1-12
    uint8_t Date;      // 1-31
    uint8_t WeekDay;   // 1-7
    uint8_t Hours;     // 0-23
    uint8_t Minutes;   // 0-59
    uint8_t Seconds;   // 0-59
    uint8_t _1s256;    // 1/256 секунды (0-255)
} RtcDataTime;
```

### 9.9 Дисплей (500)

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 500 | ssr_tft_fullscreen_repaint | ⚪ | Полная перерисовка экрана | - | - |

---

## 10. Long IO (длинные команды)

### 10.1 Протокол выполнения

См. п. 8.4. Вызов осуществляется FC=16 в область long_request/long_data (37+). При успехе long_response = OPCODE.

### 10.2 Системные команды (SystemLongRequestID)

Примечание: ✅ = реализовано, ⚪ = только определено в enum

| ID | Команда | Статус | Описание | Аргументы | Возврат |
|----|---------|--------|----------|-----------|---------|
| 0 | slr_none | ✅ | Нет операции | - | - |
| 1 | slr_get_long_io_version | ✅ | Версия Long IO | - | [1] версия (1) |
| 2 | slr_get_long_io_length | ✅ | Длина буфера Long IO | - | [1] длина (66) |
| 3 | slr_call_short_request_list_no_args_ret_1u16 | ✅ | Пакетный вызов Short IO без аргументов | [N] список ssr_ID | [N] результаты |
| 4 | slr_call_short_request_list_args_1u16_ret_1u16 | ✅ | Пакетный вызов Short IO c 1 аргументом | [2N] пары (ssr_ID,arg) | [2N] пары (response,result) |
| 5 | slr_rs485_request | ⚪ | Проксирование RS485 запроса | [N] запрос | [M] ответ |
| 6 | slr_get_int_sens_desc | ✅ | Описания внутренних датчиков | - | [N] SensorDescription[] |
| 7 | slr_replace_masked_bits | ✅ | Маскированная запись (Long) | ReplaceMaskedBitsHeader + ReplaceMaskedBitsData[] | - |
| 8 | slr_log_flash_status | ⚪ | Статус Flash-лога | - | [N] статус |
| 9 | slr_log_flash_read | ⚪ | Чтение Flash-лога | [N] параметры | [M] данные |

Структура описания датчика:
```c
typedef struct {
    uint8_t number;       // номер датчика
    uint8_t invisible;    // 0/1
    uint8_t sensor_type;  // см. sensor_type_*
    uint8_t reserved;     // резерв
} SensorDescription;
```

---

## 11. Примеры обмена

В примерах DMEM_BASE — базовый адрес окна DeviceMemoryModbus в Holding (зависит от карты регистров на стороне мастера/прибора).

### 11.1 Short IO (FC=16)

Атомарная маскированная запись бита Enable в dev_ctl:
- Запрос (старт = DMEM_BASE+17, количество слов = 1 + K):
  - short_request = 300 (ssr_replace_masked_bits)
  - short_data:
    - [0] = 3 (аргументов: header(2) + 1 пара)
    - [1] = 35 (reg_addr_start = dev_ctl)
    - [2] = 1  (num = 1 регистр)
    - [3] = 0x0001 (MASK)
    - [4] = 0x0001 (VALUE)
- Ответ:
  - short_response = 300
  - short_data[0] = 0 (нет дополнительных данных)

Пример: Получение версии Short IO
- Запрос:
  - short_request = 1
  - short_data[0] = 0
- Ответ:
  - short_response = 1
  - short_data[0] = 1
  - short_data[1] = 1

Пример: Прочитать/установить RS485 адрес
- Get:
  - short_request = 50
  - short_data[0] = 0
  - Ответ: short_response = 50; short_data = [1, адрес]
- Set:
  - short_request = 51
  - short_data = [1, новый_адрес]
  - Ответ: short_response = 51; short_data = [1, 0] (0=OK)

Пример: Чтение времени RTC0
- Запрос: short_request = 400; short_data[0] = 0
- Ответ: short_response = 400; short_data[0] = 4; далее 8 байт RtcDataTime (упаковано в 4 слова)

### 11.2 Long IO (FC=16)

Пример: Описание внутренних датчиков
- Запрос (в область long):
  - long_request = 6 (slr_get_int_sens_desc)
  - long_data[0] = 0
- Ответ:
  - long_response = 6
  - long_data[0] = N (длина результата в словах)
  - long_data[1..N] = массив SensorDescription[7] (по 4 байта на датчик = 14 слов)

---

Примечания:
- Смещение short_response фиксировано = 16 во всех версиях.
- При ошибках формат ответа может быть Modbus-исключением (уровень FC), либо кодом в полезной нагрузке согласно конкретной команде.
- Для пакетных операций используйте Long IO команды 3 и 4.