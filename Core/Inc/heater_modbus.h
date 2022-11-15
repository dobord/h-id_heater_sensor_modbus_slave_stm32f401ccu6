/*
 * heater_modbus.h
 *
 *  Created on: 6 апр. 2022 г.
 *      Author: Dobord
 */

#ifndef HEATER_MODBUS_H
#define HEATER_MODBUS_H

#include <stdint.h>

#ifdef QT_CORE_LIB
#include <QtGlobal>
#else // QT_CORE_LIB
#ifndef Q_PACKED
#define Q_PACKED __attribute__ ((__packed__))
#endif
#endif // QT_CORE_LIB

// °C

#define HID_HEATER_SENSOR_TYPE_TBL(R,X)\
    R(X,    , u 	, undefined         , ""      			, ""    )\
    R(X,    , b   	, bit               , "Бит"             , ""    )\
    R(X,    , a   	, adc_value         , "Значение АЦП"    , ""    )\
    R(X,    , U     , voltage           , "Напряжение"      , "V"  	)\
    R(X,    , I     , current           , "Сила тока"       , "A"  	)\
    R(X,    , R     , resistance        , "Сопротивление"   , "Ohm"	)\
    R(X,    , T     , temperature       , "Температура"     , "C" 	)\
    R(X,    , H     , humidity          , "Влажность"       , "%"   )\
    R(X,    , C   	, carbon_dioxide    , "CO2"             , "ppm"	)\
    R(X,    , L   	, illumination      , "Освещённость"    , "lux" )\

#define HID_HEATER_DECL_SENSOR_TYPE_ENUM(PREFIX, INIT, SYMBOL, NAME, TITLE, DIMENSION)\
    PREFIX##NAME INIT,


#define HID_HEATER_LOG_MESSAGE_TYPE_TBL(R,X)\
    R(X, none               ,       , "" )\
    R(X, power_on           ,       , "Питание появилось (включение)" )\
    R(X, power_off          ,       , "Питание пропало (выключение)" )\
    R(X, firmware_updated   ,       , "Обновление прошивки" )\
    R(X, critical           ,       , "Критический сбой" )\
    R(X, error              ,       , "Ошибка" )\
    R(X, warning            ,       , "Предупреждение" )\
    R(X, info               ,       , "Информация для справки" )\

#define HID_HEATER_LOG_MESSAGE_TYPE_ENUM(PREFIX, NAME, INIT, TITLE)\
    PREFIX##NAME INIT,


typedef enum SensorType
{
    HID_HEATER_SENSOR_TYPE_TBL(HID_HEATER_DECL_SENSOR_TYPE_ENUM,sensor_type_)
    sensor_type_num_of_types
} SensorType;

typedef enum HeaterSystemSoftwareType
{
	hsst_unknow,

	/// Устройство управления с экраном
	hsst_control,

	/// Устройство с датчиками температуры
	hsst_sensor_temperature,

} HeaterSystemSoftwareType;


typedef enum SystemShortRequestID
{
	ssr_none,
	ssr_get_short_io_version,
	ssr_get_short_io_length,
	ssr_get_long_io_version,
	ssr_get_long_io_length,
	ssr_get_software_type,
	ssr_get_device_memory_modbus_length,

	ssr_get_rs485_id = 50,
	ssr_set_rs485_id,
	ssr_get_devid,
	ssr_get_revid,
	ssr_get_uid,

	ssr_eeprom_factory_defaults = 90,
	ssr_eeprom_load_config,
	ssr_eeprom_save_config,

	ssr_get_addr_long_io = 100,
	ssr_get_addr_dev_ctl,
	ssr_get_addr_common_ctl,
	ssr_get_addr_heater_ctl,
	ssr_get_addr_heater_param,
	ssr_get_addr_heater_counter_q,

	ssr_get_addr_int_sens_status = 110,
	ssr_get_addr_int_sens_value,

	ssr_get_addr_sens_addr = 120,
	ssr_get_addr_sens_desc,
	ssr_get_addr_sens_enable,
	ssr_get_addr_sens_status,
	ssr_get_addr_sens_value,

	ssr_get_addr_relay_ctl = 130,
	ssr_get_addr_relay_status,
	ssr_get_addr_relay_value,

	ssr_get_addr_display_output_control = 140,
	ssr_get_addr_display_variable,

	ssr_get_max_num_of_internal_sensors = 200,
	ssr_get_max_num_of_internal_relays,
	ssr_get_max_num_of_sensors,
	ssr_get_max_num_of_relays,
	ssr_get_max_num_of_heater_ctl,
	ssr_get_max_num_of_common_ctl,
	ssr_get_max_num_of_display_variable,

	ssr_replace_masked_bits = 300,

	ssr_rtc0_read_date_time = 400,
	ssr_rtc0_write_date_time,

	ssr_rtc1_read_date_time = 420,
	ssr_rtc1_write_date_time,
	ssr_rtc1_read_calibr,
	ssr_rtc1_write_calibr,
	ssr_rtc1_read_regs,
	ssr_rtc1_write_regs,
    ssr_rtc1_copy_to_rtc0,

    /// обновить изображение экрана с очисткой
    ssr_tft_fullscreen_repaint = 500,
} SystemShortRequestID;

typedef enum SystemLongRequestID
{
	slr_none,
	slr_get_long_io_version,
	slr_get_long_io_length,
	slr_call_short_request_list_no_args_ret_1u16,
	slr_call_short_request_list_args_1u16_ret_1u16,
	slr_rs485_request,
	slr_get_int_sens_desc,
    slr_replace_masked_bits,
    slr_log_flash_status,
    slr_log_flash_read
} SystemLongRequestID;


typedef enum HeaterConstants
{
    hc_addr_internal_relay = 0,

    hc_addr_short_io = 16,
    hc_addr_short_io_response = hc_addr_short_io,
    hc_addr_short_io_request,
    hc_addr_short_io_data,
} HeaterConstants;

typedef enum HeaterVariableContainer
{
	hvc_none,
	hvc_heater_counter_q,
	hvc_relay_pin,
	hvc_sens_value
} HeaterVariableContainer;

typedef enum LogMessageType
{
    HID_HEATER_LOG_MESSAGE_TYPE_TBL(HID_HEATER_LOG_MESSAGE_TYPE_ENUM,lmt_)
    lmt_num_of_types
} LogMessageType;

typedef enum LogStorageType
{
    /// Внутренняя ОЗУ
    lst_internal_ram,

    /// Внешняя EEPROM
    /// (AT24C32)
    lst_external_eeprom,

    /// Внешняя Flash
    /// (W25Q32A)
    lst_external_flash
} LogStorageType;

typedef union UnionRegister {
	uint8_t  ur8[8];
	uint16_t ur16[4];
	uint32_t ur32[2];
	uint64_t ur64[1];
} UnionRegister;


typedef struct SensorTypeDesc
{
	/// Тип
	uint8_t type;

	/// Символ
	const char* symbol;

	/// Название
	const char* name;

	/// Описание
	const char* title;

	/// Размерность
	const char* dimension;
} SensorTypeDesc;


typedef struct RemoteRegisterAddress
{
    /// Адрес регистра в памяти устроства
    uint8_t reg_addr;

    /// Адрес устройства в modbus на шине rs485
    uint8_t rs485_addr;
} RemoteRegisterAddress;


typedef struct RemoteRelayControl
{
    /// Номер регистра реле в памяти ведущего устройства
    uint8_t src_relay_reg: 4;

    /// Номер регистра реле в памяти ведомого устройства
    uint8_t dest_reg_addr: 4;

    /// Адрес ведомого устройства в modbus на шине rs485
    uint8_t rs485_addr;

    /// Битовая маска выбора обновляемых битов в ведомом устройстве
    /// пустая маска (=0) означает полностью отключенную логику
    uint16_t dest_mask;

    /// Битовая маска инвертирования выбранных битов
    uint16_t dest_invert;

    /// Битовая маска выбора битов для установки в 0 (в ведомом устройстве)
    /// при условии разрешения бита получателя в dest_mask и сброшеном бите в dest_ones
    uint16_t dest_zeros;

    /// Битовая маска выбора битов для установки в 1 (в ведомом устройстве)
    /// при условии разрешения бита получателя в dest_mask и сброшеном бите в dest_zeros
    uint16_t dest_ones;

    /// Регистр выбора бита источника, для каждого бита получателя
    /// b3..b0 - номер бита источника для 0-ого бита получателя
    /// b7..b4 - номер бита источника для 1-ого бита получателя
    /// и.т.д.
    /// применяется к битам при условии разрешения бита получателя в dest_mask
    /// и установленых одновременно битах в dest_zeros и dest_ones
    uint16_t bit_src_mixer[4];
} RemoteRelayControl;



typedef struct SensorDescription
{
    /// Номер датчика,
    /// уникальный в группе датчиков одного типа (@sa sensor_type)
    uint8_t number :7;

    /// Признак сиситемного датчика
    /// недоступного для пользователя
    uint8_t invisible :1;

    /// Тип датчика
    /// @sa SensorType
    uint8_t sensor_type;

} SensorDescription;


/// Параметры управления насосом солнечного нагревателя
typedef struct SolarHeaterControl
{
    /// Разрешение работы
    uint16_t enable: 1;
    uint16_t : 7;

    /// Номер регистра Т1
    uint16_t t1: 8;


    /// Номер регистра Т2
    uint16_t t2: 8;

    /// Номер бита вывода реле
    uint16_t n: 8;


    /// Пороговое значение разницы (Т1 - Т2) для включения
    int16_t dt_on;


    /// Пороговое значение разницы (Т1 - Т2) для выключения
    int16_t dt_off;

} SolarHeaterControl;


// 1Гкал = 1.163 МВт*ч
// Удельная теплоёмкость воды, кДж/(кг*К) =	4.1868


/// Параметры расчета полученого тепла от солнечного нагревателя
typedef struct SolarHeaterParams
{
	/// Производительность насоса.
	/// объем прокачеваемой жидкости за минуту (литров в минуту)
	uint8_t pump_performance;

	/// Удельная плотность жидкости 1ед = 0.01 кг/л
	uint8_t ro;

	/// Удельная теплоёмкость жидкости 1ед = 0.1 * Дж/(кг*К)
	uint16_t c;

} SolarHeaterParams;


/// Параметры управления обычной нагрузкой по пороговому значению
typedef struct CommonControl
{
    /// Разрешение работы
    uint16_t enable: 1;

    /// Номер регистра Т
    uint16_t t: 7;

    /// Номер бита вывода реле
    uint16_t n: 8;


    /// Пороговое значение для включения
    int16_t t_on;


    /// Дельта для выключения.
    /// При значении (t_on - T) > dt_off срабатывает выключение
    int16_t dt_off;

} CommonControl;


/// Регистр управления параметрами отображения на TFT
typedef struct DisplayOutputControl
{
	/// Всего отображаемых переменных DisplayVariable
	uint8_t num_of_vars;

	/// Кол-во основных переменных
	uint8_t num_of_main_vars;
} DisplayOutputControl;


/// Настройки отображния переменной на TFT
typedef struct DisplayVariable
{
	/// Порядковый номер для переменной внутри контейнера container
	uint8_t n;

	/// @sa HeaterVariableContainer
	uint8_t container;
} DisplayVariable;


typedef struct DeviceControlRegister_Control
{
	uint16_t rs485_io :1;
	uint16_t :1;
	uint16_t led_red: 1;
	uint16_t led_green: 1;
	uint16_t led_blue: 1;
	uint16_t led_orange: 1;
	uint16_t debug_int_relay_to_led: 1;
	uint16_t debug_sys_poll_off: 1;
	uint16_t debug_sys_ctl_off: 1;
	uint16_t debug_sys_write_off: 1;
	uint16_t debug_sys_usb_to_rs485: 1;
	uint16_t : 5;
} DeviceControlRegister_Control;


typedef struct DeviceControlRegister_SensorTemperature
{
	uint16_t rs485_io :1;
	uint16_t :1;
	uint16_t led_red: 1;
	uint16_t led_green: 1;
	uint16_t led_blue: 1;
	uint16_t : 1;
	uint16_t debug_int_relay_to_led: 1;
	uint16_t : 9;
} DeviceControlRegister_SensorTemperature;



typedef struct ReplaceMaskedBitsHeader
{
	/// Начальный адрес регистра получателя
	uint8_t reg_addr_start;

	/// Кол-во регистров
	uint8_t num;
} ReplaceMaskedBitsHeader;

typedef struct ReplaceMaskedBitsData
{
	/// Данные
	uint16_t data;

	/// Битовая маска заменяемых битов
	uint16_t mask;
} ReplaceMaskedBitsData;

typedef struct RtcDataTime
{
	// дробная часть секунды *1s/256
	uint8_t _1s256;
	uint8_t Seconds;
	uint8_t Minutes;
	uint8_t Hours;
	uint8_t WeekDay;
	uint8_t Date;
	uint8_t Month;
	uint8_t Year;
} RtcDataTime;

typedef struct RtcRegsHeader
{
	// размер (в байтах)
	uint8_t size;

	// смещение (в байтах)
	uint8_t offset;
} RtcRegsHeader;

typedef struct SlrRs485RequestHeader
{
    uint8_t u8id;          /*!< Slave address between 1 and 247. 0 means broadcast */
    uint8_t u8fct;         /*!< Function code: 1, 2, 3, 4, 5, 6, 15 or 16 */
    uint16_t u16RegAdd;    /*!< Address of the first register to access at slave/s */
    uint16_t u16CoilsNo;   /*!< Number of coils or registers to access */
} SlrRs485RequestHeader;


#pragma pack(push,1)
typedef struct Q_PACKED LogHeader
{
    /// Размер (в байтах)
    /// вместе с заголовком
    uint8_t length;

    /// Тип
    ///  @sa LogMessageType
    uint8_t type;

    /// Дата и время в формате Unix timestamp UTC
    uint32_t timestamp;
} LogHeader;
#pragma pack(pop)


typedef struct SlrLogReadHeader
{
    /// Размер вычитывемых данных
    uint32_t size: 8;

    /// Начальный адрес лога
    uint32_t addr: 24;
} SlrLogReadHeader;

typedef struct SlrLogStatus
{
    uint32_t virt_top;
    uint32_t size;
    uint32_t capacity;
} SlrLogStatus;

#endif // HEATER_MODBUS_H
