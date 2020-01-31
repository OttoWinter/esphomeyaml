"""Constants used by esphome."""

MAJOR_VERSION = 1
MINOR_VERSION = 15
PATCH_VERSION = '0-dev'
__short_version__ = f'{MAJOR_VERSION}.{MINOR_VERSION}'
__version__ = f'{__short_version__}.{PATCH_VERSION}'

ESP_PLATFORM_ESP32 = 'ESP32'
ESP_PLATFORM_ESP8266 = 'ESP8266'
ESP_PLATFORMS = [ESP_PLATFORM_ESP32, ESP_PLATFORM_ESP8266]

ALLOWED_NAME_CHARS = 'abcdefghijklmnopqrstuvwxyz0123456789_'
ARDUINO_VERSION_ESP32_DEV = 'https://github.com/platformio/platform-espressif32.git#feature/stage'
ARDUINO_VERSION_ESP32_1_0_0 = 'espressif32@1.5.0'
ARDUINO_VERSION_ESP32_1_0_1 = 'espressif32@1.6.0'
ARDUINO_VERSION_ESP32_1_0_2 = 'espressif32@1.9.0'
ARDUINO_VERSION_ESP32_1_0_3 = 'espressif32@1.10.0'
ARDUINO_VERSION_ESP32_1_0_4 = 'espressif32@1.11.0'
ARDUINO_VERSION_ESP8266_DEV = 'https://github.com/platformio/platform-espressif8266.git#feature' \
                              '/stage'
ARDUINO_VERSION_ESP8266_2_5_0 = 'espressif8266@2.0.1'
ARDUINO_VERSION_ESP8266_2_5_1 = 'espressif8266@2.1.0'
ARDUINO_VERSION_ESP8266_2_5_2 = 'espressif8266@2.2.3'
ARDUINO_VERSION_ESP8266_2_3_0 = 'espressif8266@1.5.0'
SOURCE_FILE_EXTENSIONS = {'.cpp', '.hpp', '.h', '.c', '.tcc', '.ino'}
HEADER_FILE_EXTENSIONS = {'.h', '.hpp', '.tcc'}

CONF_ABOVE = 'above'
CONF_ABSOLUTE_COUNT_MODE = 'absolute_count_mode'
CONF_ACCELERATION = 'acceleration'
CONF_ACCELERATION_X = 'acceleration_x'
CONF_ACCELERATION_Y = 'acceleration_y'
CONF_ACCELERATION_Z = 'acceleration_z'
CONF_ACCURACY = 'accuracy'
CONF_ACCURACY_DECIMALS = 'accuracy_decimals'
CONF_ACTION_ID = 'action_id'
CONF_ADDRESS = 'address'
CONF_ALPHA = 'alpha'
CONF_AND = 'and'
CONF_AP = 'ap'
CONF_ARDUINO_VERSION = 'arduino_version'
CONF_ARGS = 'args'
CONF_ASSUMED_STATE = 'assumed_state'
CONF_AT = 'at'
CONF_ATTENUATION = 'attenuation'
CONF_AUTH = 'auth'
CONF_AUTOMATION_ID = 'automation_id'
CONF_AVAILABILITY = 'availability'
CONF_AWAY = 'away'
CONF_AWAY_CONFIG = 'away_config'
CONF_BATTERY_LEVEL = 'battery_level'
CONF_BATTERY_VOLTAGE = 'battery_voltage'
CONF_BAUD_RATE = 'baud_rate'
CONF_BELOW = 'below'
CONF_BINARY = 'binary'
CONF_BINARY_SENSOR = 'binary_sensor'
CONF_BINARY_SENSORS = 'binary_sensors'
CONF_BIRTH_MESSAGE = 'birth_message'
CONF_BIT_DEPTH = 'bit_depth'
CONF_BLUE = 'blue'
CONF_BOARD = 'board'
CONF_BOARD_FLASH_MODE = 'board_flash_mode'
CONF_BRANCH = 'branch'
CONF_BRIGHTNESS = 'brightness'
CONF_BROKER = 'broker'
CONF_BSSID = 'bssid'
CONF_BUFFER_SIZE = 'buffer_size'
CONF_BUILD_PATH = 'build_path'
CONF_BUS_VOLTAGE = 'bus_voltage'
CONF_BUSY_PIN = 'busy_pin'
CONF_CALIBRATE_LINEAR = 'calibrate_linear'
CONF_CALIBRATION = 'calibration'
CONF_CAPACITANCE = 'capacitance'
CONF_CARRIER_DUTY_PERCENT = 'carrier_duty_percent'
CONF_CARRIER_FREQUENCY = 'carrier_frequency'
CONF_CHANGE_MODE_EVERY = 'change_mode_every'
CONF_CHANNEL = 'channel'
CONF_CHANNELS = 'channels'
CONF_CHIPSET = 'chipset'
CONF_CLIENT_ID = 'client_id'
CONF_CLK_PIN = 'clk_pin'
CONF_CLOCK_PIN = 'clock_pin'
CONF_CLOSE_ACTION = 'close_action'
CONF_CLOSE_DURATION = 'close_duration'
CONF_CLOSE_ENDSTOP = 'close_endstop'
CONF_CO2 = 'co2'
CONF_CODE = 'code'
CONF_COLD_WHITE = 'cold_white'
CONF_COLD_WHITE_COLOR_TEMPERATURE = 'cold_white_color_temperature'
CONF_COLOR_CORRECT = 'color_correct'
CONF_COLOR_TEMPERATURE = 'color_temperature'
CONF_COLORS = 'colors'
CONF_COMMAND = 'command'
CONF_COMMAND_TOPIC = 'command_topic'
CONF_COMMENT = 'comment'
CONF_COMMIT = 'commit'
CONF_COMPONENT_ID = 'component_id'
CONF_COMPONENTS = 'components'
CONF_CONDITION = 'condition'
CONF_CONDITION_ID = 'condition_id'
CONF_CONDUCTIVITY = 'conductivity'
CONF_COOL_ACTION = 'cool_action'
CONF_COUNT_MODE = 'count_mode'
CONF_CRON = 'cron'
CONF_CS_PIN = 'cs_pin'
CONF_CSS_URL = 'css_url'
CONF_CURRENT = 'current'
CONF_CURRENT_OPERATION = 'current_operation'
CONF_CURRENT_RESISTOR = 'current_resistor'
CONF_DALLAS_ID = 'dallas_id'
CONF_DATA = 'data'
CONF_DATA_PIN = 'data_pin'
CONF_DATA_PINS = 'data_pins'
CONF_DATA_TEMPLATE = 'data_template'
CONF_DAYS_OF_MONTH = 'days_of_month'
CONF_DAYS_OF_WEEK = 'days_of_week'
CONF_DC_PIN = 'dc_pin'
CONF_DEBOUNCE = 'debounce'
CONF_DECELERATION = 'deceleration'
CONF_DEFAULT_TARGET_TEMPERATURE_HIGH = 'default_target_temperature_high'
CONF_DEFAULT_TARGET_TEMPERATURE_LOW = 'default_target_temperature_low'
CONF_DEFAULT_TRANSITION_LENGTH = 'default_transition_length'
CONF_DELAY = 'delay'
CONF_DELTA = 'delta'
CONF_DEVICE = 'device'
CONF_DEVICE_CLASS = 'device_class'
CONF_DIMENSIONS = 'dimensions'
CONF_DIO_PIN = 'dio_pin'
CONF_DIR_PIN = 'dir_pin'
CONF_DIRECTION = 'direction'
CONF_DISCOVERY = 'discovery'
CONF_DISCOVERY_PREFIX = 'discovery_prefix'
CONF_DISCOVERY_RETAIN = 'discovery_retain'
CONF_DISTANCE = 'distance'
CONF_DIV_RATIO = 'div_ratio'
CONF_DNS1 = 'dns1'
CONF_DNS2 = 'dns2'
CONF_DOMAIN = 'domain'
CONF_DUMP = 'dump'
CONF_DURATION = 'duration'
CONF_ECHO_PIN = 'echo_pin'
CONF_EFFECT = 'effect'
CONF_EFFECTS = 'effects'
CONF_ELSE = 'else'
CONF_ENABLE_PIN = 'enable_pin'
CONF_ENABLE_TIME = 'enable_time'
CONF_ENTITY_ID = 'entity_id'
CONF_ESP8266_RESTORE_FROM_FLASH = 'esp8266_restore_from_flash'
CONF_ESPHOME = 'esphome'
CONF_ESPHOME_CORE_VERSION = 'esphome_core_version'
CONF_EVENT = 'event'
CONF_EXPIRE_AFTER = 'expire_after'
CONF_EXTERNAL_VCC = 'external_vcc'
CONF_FALLING_EDGE = 'falling_edge'
CONF_FAMILY = 'family'
CONF_FAN_MODE = 'fan_mode'
CONF_FAST_CONNECT = 'fast_connect'
CONF_FILE = 'file'
CONF_FILTER = 'filter'
CONF_FILTER_OUT = 'filter_out'
CONF_FILTERS = 'filters'
CONF_FLASH_LENGTH = 'flash_length'
CONF_FOR = 'for'
CONF_FORCE_UPDATE = 'force_update'
CONF_FORMALDEHYDE = 'formaldehyde'
CONF_FORMAT = 'format'
CONF_FREQUENCY = 'frequency'
CONF_FROM = 'from'
CONF_FULL_UPDATE_EVERY = 'full_update_every'
CONF_GAIN = 'gain'
CONF_GAMMA_CORRECT = 'gamma_correct'
CONF_GAS_RESISTANCE = 'gas_resistance'
CONF_GATEWAY = 'gateway'
CONF_GLYPHS = 'glyphs'
CONF_GPIO = 'gpio'
CONF_GREEN = 'green'
CONF_GROUP = 'group'
CONF_HARDWARE_UART = 'hardware_uart'
CONF_HEARTBEAT = 'heartbeat'
CONF_HEAT_ACTION = 'heat_action'
CONF_HEATER = 'heater'
CONF_HIDDEN = 'hidden'
CONF_HIGH = 'high'
CONF_HIGH_VOLTAGE_REFERENCE = 'high_voltage_reference'
CONF_HOUR = 'hour'
CONF_HOURS = 'hours'
CONF_HUMIDITY = 'humidity'
CONF_I2C = 'i2c'
CONF_I2C_ID = 'i2c_id'
CONF_ICON = 'icon'
CONF_ID = 'id'
CONF_IDLE = 'idle'
CONF_IDLE_ACTION = 'idle_action'
CONF_IDLE_LEVEL = 'idle_level'
CONF_IF = 'if'
CONF_IIR_FILTER = 'iir_filter'
CONF_ILLUMINANCE = 'illuminance'
CONF_INCLUDES = 'includes'
CONF_INDEX = 'index'
CONF_INDOOR = 'indoor'
CONF_INITIAL_MODE = 'initial_mode'
CONF_INITIAL_VALUE = 'initial_value'
CONF_INTEGRATION_TIME = 'integration_time'
CONF_INTENSITY = 'intensity'
CONF_INTERLOCK = 'interlock'
CONF_INTERNAL = 'internal'
CONF_INTERNAL_FILTER = 'internal_filter'
CONF_INTERVAL = 'interval'
CONF_INVALID_COOLDOWN = 'invalid_cooldown'
CONF_INVERT = 'invert'
CONF_INVERTED = 'inverted'
CONF_IP_ADDRESS = 'ip_address'
CONF_JS_URL = 'js_url'
CONF_JVC = 'jvc'
CONF_KEEP_ON_TIME = 'keep_on_time'
CONF_KEEPALIVE = 'keepalive'
CONF_LAMBDA = 'lambda'
CONF_LEVEL = 'level'
CONF_LG = 'lg'
CONF_LIBRARIES = 'libraries'
CONF_LIGHT = 'light'
CONF_LIGHTNING_ENERGY = 'lightning_energy'
CONF_LIGHTNING_THRESHOLD = 'lightning_threshold'
CONF_LOADED_INTEGRATIONS = 'loaded_integrations'
CONF_LOCAL = 'local'
CONF_LOG_TOPIC = 'log_topic'
CONF_LOGGER = 'logger'
CONF_LOGS = 'logs'
CONF_LOW = 'low'
CONF_LOW_VOLTAGE_REFERENCE = 'low_voltage_reference'
CONF_MAC_ADDRESS = 'mac_address'
CONF_MAINS_FILTER = 'mains_filter'
CONF_MAKE_ID = 'make_id'
CONF_MANUAL_IP = 'manual_ip'
CONF_MASK_DISTURBER = 'mask_disturber'
CONF_MAX_CURRENT = 'max_current'
CONF_MAX_DURATION = 'max_duration'
CONF_MAX_LENGTH = 'max_length'
CONF_MAX_LEVEL = 'max_level'
CONF_MAX_POWER = 'max_power'
CONF_MAX_REFRESH_RATE = 'max_refresh_rate'
CONF_MAX_SPEED = 'max_speed'
CONF_MAX_TEMPERATURE = 'max_temperature'
CONF_MAX_VALUE = 'max_value'
CONF_MAX_VOLTAGE = 'max_voltage'
CONF_MEASUREMENT_DURATION = 'measurement_duration'
CONF_MEASUREMENT_SEQUENCE_NUMBER = 'measurement_sequence_number'
CONF_MEDIUM = 'medium'
CONF_METHOD = 'method'
CONF_MIN_LENGTH = 'min_length'
CONF_MIN_LEVEL = 'min_level'
CONF_MIN_POWER = 'min_power'
CONF_MIN_TEMPERATURE = 'min_temperature'
CONF_MIN_VALUE = 'min_value'
CONF_MINUTE = 'minute'
CONF_MINUTES = 'minutes'
CONF_MISO_PIN = 'miso_pin'
CONF_MODE = 'mode'
CONF_MODEL = 'model'
CONF_MOISTURE = 'moisture'
CONF_MONTHS = 'months'
CONF_MOSI_PIN = 'mosi_pin'
CONF_MOVEMENT_COUNTER = 'movement_counter'
CONF_MQTT = 'mqtt'
CONF_MQTT_ID = 'mqtt_id'
CONF_MULTIPLEXER = 'multiplexer'
CONF_MULTIPLY = 'multiply'
CONF_NAME = 'name'
CONF_NBITS = 'nbits'
CONF_NEC = 'nec'
CONF_NETWORKS = 'networks'
CONF_NOISE_LEVEL = 'noise_level'
CONF_NUM_ATTEMPTS = 'num_attempts'
CONF_NUM_CHANNELS = 'num_channels'
CONF_NUM_CHIPS = 'num_chips'
CONF_NUM_LEDS = 'num_leds'
CONF_NUMBER = 'number'
CONF_OFFSET = 'offset'
CONF_ON = 'on'
CONF_ON_BOOT = 'on_boot'
CONF_ON_CLICK = 'on_click'
CONF_ON_DOUBLE_CLICK = 'on_double_click'
CONF_ON_JSON_MESSAGE = 'on_json_message'
CONF_ON_LOOP = 'on_loop'
CONF_ON_MESSAGE = 'on_message'
CONF_ON_MULTI_CLICK = 'on_multi_click'
CONF_ON_PRESS = 'on_press'
CONF_ON_RAW_VALUE = 'on_raw_value'
CONF_ON_RELEASE = 'on_release'
CONF_ON_SHUTDOWN = 'on_shutdown'
CONF_ON_STATE = 'on_state'
CONF_ON_TAG = 'on_tag'
CONF_ON_TIME = 'on_time'
CONF_ON_TURN_OFF = 'on_turn_off'
CONF_ON_TURN_ON = 'on_turn_on'
CONF_ON_VALUE = 'on_value'
CONF_ON_VALUE_RANGE = 'on_value_range'
CONF_ONE = 'one'
CONF_OPEN_ACTION = 'open_action'
CONF_OPEN_DURATION = 'open_duration'
CONF_OPEN_ENDSTOP = 'open_endstop'
CONF_OPTIMISTIC = 'optimistic'
CONF_OR = 'or'
CONF_OSCILLATING = 'oscillating'
CONF_OSCILLATION_COMMAND_TOPIC = 'oscillation_command_topic'
CONF_OSCILLATION_OUTPUT = 'oscillation_output'
CONF_OSCILLATION_STATE_TOPIC = 'oscillation_state_topic'
CONF_OTA = 'ota'
CONF_OUTPUT = 'output'
CONF_OUTPUT_ID = 'output_id'
CONF_OUTPUTS = 'outputs'
CONF_OVERSAMPLING = 'oversampling'
CONF_PAGE_ID = 'page_id'
CONF_PAGES = 'pages'
CONF_PANASONIC = 'panasonic'
CONF_PASSWORD = 'password'
CONF_PAYLOAD = 'payload'
CONF_PAYLOAD_AVAILABLE = 'payload_available'
CONF_PAYLOAD_NOT_AVAILABLE = 'payload_not_available'
CONF_PERIOD = 'period'
CONF_PHASE_BALANCER = 'phase_balancer'
CONF_PIN = 'pin'
CONF_PIN_A = 'pin_a'
CONF_PIN_B = 'pin_b'
CONF_PIN_C = 'pin_c'
CONF_PIN_D = 'pin_d'
CONF_PINS = 'pins'
CONF_PLATFORM = 'platform'
CONF_PLATFORMIO_OPTIONS = 'platformio_options'
CONF_PM_1_0 = 'pm_1_0'
CONF_PM_10_0 = 'pm_10_0'
CONF_PM_2_5 = 'pm_2_5'
CONF_PM_4_0 = 'pm_4_0'
CONF_PM_SIZE = 'pm_size'
CONF_PMC_0_5 = 'pmc_0_5'
CONF_PMC_1_0 = 'pmc_1_0'
CONF_PMC_10_0 = 'pmc_10_0'
CONF_PMC_2_5 = 'pmc_2_5'
CONF_PMC_4_0 = 'pmc_4_0'
CONF_PORT = 'port'
CONF_POSITION = 'position'
CONF_POSITION_ACTION = 'position_action'
CONF_POWER = 'power'
CONF_POWER_FACTOR = 'power_factor'
CONF_POWER_ON_VALUE = 'power_on_value'
CONF_POWER_SAVE_MODE = 'power_save_mode'
CONF_POWER_SUPPLY = 'power_supply'
CONF_PRESSURE = 'pressure'
CONF_PRIORITY = 'priority'
CONF_PROTOCOL = 'protocol'
CONF_PULL_MODE = 'pull_mode'
CONF_PULSE_LENGTH = 'pulse_length'
CONF_QOS = 'qos'
CONF_RANDOM = 'random'
CONF_RANGE = 'range'
CONF_RANGE_FROM = 'range_from'
CONF_RANGE_TO = 'range_to'
CONF_RATE = 'rate'
CONF_RAW = 'raw'
CONF_RC_CODE_1 = 'rc_code_1'
CONF_RC_CODE_2 = 'rc_code_2'
CONF_REBOOT_TIMEOUT = 'reboot_timeout'
CONF_RECEIVE_TIMEOUT = 'receive_timeout'
CONF_RED = 'red'
CONF_REFERENCE_RESISTANCE = 'reference_resistance'
CONF_REFERENCE_TEMPERATURE = 'reference_temperature'
CONF_REPEAT = 'repeat'
CONF_REPOSITORY = 'repository'
CONF_RESET_PIN = 'reset_pin'
CONF_RESIZE = 'resize'
CONF_RESOLUTION = 'resolution'
CONF_RESTORE = 'restore'
CONF_RESTORE_MODE = 'restore_mode'
CONF_RESTORE_STATE = 'restore_state'
CONF_RESTORE_VALUE = 'restore_value'
CONF_RETAIN = 'retain'
CONF_RGB_ORDER = 'rgb_order'
CONF_RGBW = 'rgbw'
CONF_RISING_EDGE = 'rising_edge'
CONF_ROTATION = 'rotation'
CONF_RS_PIN = 'rs_pin'
CONF_RTD_NOMINAL_RESISTANCE = 'rtd_nominal_resistance'
CONF_RTD_WIRES = 'rtd_wires'
CONF_RUN_CYCLES = 'run_cycles'
CONF_RUN_DURATION = 'run_duration'
CONF_RW_PIN = 'rw_pin'
CONF_RX_ONLY = 'rx_only'
CONF_RX_PIN = 'rx_pin'
CONF_SAFE_MODE = 'safe_mode'
CONF_SAMSUNG = 'samsung'
CONF_SCAN = 'scan'
CONF_SCL = 'scl'
CONF_SCL_PIN = 'scl_pin'
CONF_SDA = 'sda'
CONF_SDO_PIN = 'sdo_pin'
CONF_SECOND = 'second'
CONF_SECONDS = 'seconds'
CONF_SEGMENTS = 'segments'
CONF_SEL_PIN = 'sel_pin'
CONF_SEND_EVERY = 'send_every'
CONF_SEND_FIRST_AT = 'send_first_at'
CONF_SENSOR = 'sensor'
CONF_SENSOR_ID = 'sensor_id'
CONF_SENSORS = 'sensors'
CONF_SEQUENCE = 'sequence'
CONF_SERVERS = 'servers'
CONF_SERVICE = 'service'
CONF_SERVICE_UUID = 'service_uuid'
CONF_SERVICES = 'services'
CONF_SETUP_MODE = 'setup_mode'
CONF_SETUP_PRIORITY = 'setup_priority'
CONF_SHUNT_RESISTANCE = 'shunt_resistance'
CONF_SHUNT_VOLTAGE = 'shunt_voltage'
CONF_SHUTDOWN_MESSAGE = 'shutdown_message'
CONF_SIZE = 'size'
CONF_SLEEP_DURATION = 'sleep_duration'
CONF_SLEEP_PIN = 'sleep_pin'
CONF_SLEEP_WHEN_DONE = 'sleep_when_done'
CONF_SONY = 'sony'
CONF_SPEED = 'speed'
CONF_SPEED_COMMAND_TOPIC = 'speed_command_topic'
CONF_SPEED_STATE_TOPIC = 'speed_state_topic'
CONF_SPI_ID = 'spi_id'
CONF_SPIKE_REJECTION = 'spike_rejection'
CONF_SSID = 'ssid'
CONF_SSL_FINGERPRINTS = 'ssl_fingerprints'
CONF_STATE = 'state'
CONF_STATE_TOPIC = 'state_topic'
CONF_STATIC_IP = 'static_ip'
CONF_STEP_MODE = 'step_mode'
CONF_STEP_PIN = 'step_pin'
CONF_STOP = 'stop'
CONF_STOP_ACTION = 'stop_action'
CONF_SUBNET = 'subnet'
CONF_SUPPORTS_COOL = 'supports_cool'
CONF_SUPPORTS_HEAT = 'supports_heat'
CONF_SWING_MODE = 'swing_mode'
CONF_SWITCHES = 'switches'
CONF_SYNC = 'sync'
CONF_TAG = 'tag'
CONF_TARGET = 'target'
CONF_TARGET_TEMPERATURE = 'target_temperature'
CONF_TARGET_TEMPERATURE_HIGH = 'target_temperature_high'
CONF_TARGET_TEMPERATURE_LOW = 'target_temperature_low'
CONF_TEMPERATURE = 'temperature'
CONF_TEMPERATURE_STEP = 'temperature_step'
CONF_TEXT_SENSORS = 'text_sensors'
CONF_THEN = 'then'
CONF_THRESHOLD = 'threshold'
CONF_THROTTLE = 'throttle'
CONF_TILT = 'tilt'
CONF_TILT_ACTION = 'tilt_action'
CONF_TILT_LAMBDA = 'tilt_lambda'
CONF_TIME = 'time'
CONF_TIME_ID = 'time_id'
CONF_TIMEOUT = 'timeout'
CONF_TIMES = 'times'
CONF_TIMEZONE = 'timezone'
CONF_TIMING = 'timing'
CONF_TO = 'to'
CONF_TOLERANCE = 'tolerance'
CONF_TOPIC = 'topic'
CONF_TOPIC_PREFIX = 'topic_prefix'
CONF_TRANSITION_LENGTH = 'transition_length'
CONF_TRIGGER_ID = 'trigger_id'
CONF_TRIGGER_PIN = 'trigger_pin'
CONF_TURN_OFF_ACTION = 'turn_off_action'
CONF_TURN_ON_ACTION = 'turn_on_action'
CONF_TX_BUFFER_SIZE = 'tx_buffer_size'
CONF_TX_PIN = 'tx_pin'
CONF_TX_POWER = 'tx_power'
CONF_TYPE = 'type'
CONF_TYPE_ID = 'type_id'
CONF_UART_ID = 'uart_id'
CONF_UID = 'uid'
CONF_UNIQUE = 'unique'
CONF_UNIT_OF_MEASUREMENT = 'unit_of_measurement'
CONF_UPDATE_INTERVAL = 'update_interval'
CONF_UPDATE_ON_BOOT = 'update_on_boot'
CONF_USE_ADDRESS = 'use_address'
CONF_USERNAME = 'username'
CONF_UUID = 'uuid'
CONF_VALUE = 'value'
CONF_VARIABLES = 'variables'
CONF_VARIANT = 'variant'
CONF_VISUAL = 'visual'
CONF_VOLTAGE = 'voltage'
CONF_VOLTAGE_ATTENUATION = 'voltage_attenuation'
CONF_VOLTAGE_DIVIDER = 'voltage_divider'
CONF_WAIT_TIME = 'wait_time'
CONF_WAIT_UNTIL = 'wait_until'
CONF_WAKEUP_PIN = 'wakeup_pin'
CONF_WARM_WHITE = 'warm_white'
CONF_WARM_WHITE_COLOR_TEMPERATURE = 'warm_white_color_temperature'
CONF_WATCHDOG_THRESHOLD = 'watchdog_threshold'
CONF_WHILE = 'while'
CONF_WHITE = 'white'
CONF_WIDTH = 'width'
CONF_WIFI = 'wifi'
CONF_WILL_MESSAGE = 'will_message'
CONF_WIND_DIRECTION_DEGREES = 'wind_direction_degrees'
CONF_WIND_SPEED = 'wind_speed'
CONF_WINDOW_SIZE = 'window_size'
CONF_ZERO = 'zero'

ICON_ACCELERATION = 'mdi:axis-arrow'
ICON_ACCELERATION_X = 'mdi:axis-x-arrow'
ICON_ACCELERATION_Y = 'mdi:axis-y-arrow'
ICON_ACCELERATION_Z = 'mdi:axis-z-arrow'
ICON_ARROW_EXPAND_VERTICAL = 'mdi:arrow-expand-vertical'
ICON_BATTERY = 'mdi:battery'
ICON_BRIEFCASE_DOWNLOAD = 'mdi:briefcase-download'
ICON_BRIGHTNESS_5 = 'mdi:brightness-5'
ICON_CHECK_CIRCLE_OUTLINE = 'mdi:check-circle-outline'
ICON_CHEMICAL_WEAPON = 'mdi:chemical-weapon'
ICON_COUNTER = 'mdi:counter'
ICON_CURRENT_AC = 'mdi:current-ac'
ICON_EMPTY = ''
ICON_FLASH = 'mdi:flash'
ICON_FLOWER = 'mdi:flower'
ICON_GAS_CYLINDER = 'mdi:gas-cylinder'
ICON_GAUGE = 'mdi:gauge'
ICON_LIGHTBULB = 'mdi:lightbulb'
ICON_MAGNET = 'mdi:magnet'
ICON_NEW_BOX = 'mdi:new-box'
ICON_PERCENT = 'mdi:percent'
ICON_PERIODIC_TABLE_CO2 = 'mdi:periodic-table-co2'
ICON_POWER = 'mdi:power'
ICON_PULSE = 'mdi:pulse'
ICON_RADIATOR = 'mdi:radiator'
ICON_RESTART = 'mdi:restart'
ICON_ROTATE_RIGHT = 'mdi:rotate-right'
ICON_RULER = 'mdi:ruler'
ICON_SCALE = 'mdi:scale'
ICON_SCREEN_ROTATION = 'mdi:screen-rotation'
ICON_SIGN_DIRECTION = 'mdi:sign-direction'
ICON_SIGNAL = 'mdi:signal-distance-variant'
ICON_SIGNAL_DISTANCE_VARIANT = 'mdi:signal'
ICON_THERMOMETER = 'mdi:thermometer'
ICON_TIMER = 'mdi:timer'
ICON_WATER_PERCENT = 'mdi:water-percent'
ICON_WEATHER_SUNSET = 'mdi:weather-sunset'
ICON_WEATHER_SUNSET_DOWN = 'mdi:weather-sunset-down'
ICON_WEATHER_SUNSET_UP = 'mdi:weather-sunset-up'
ICON_WEATHER_WINDY = 'mdi:weather-windy'
ICON_WIFI = 'mdi:wifi'

UNIT_AMPERE = 'A'
UNIT_CELSIUS = '°C'
UNIT_COUNTS_PER_CUBIC_METER = '#/m³'
UNIT_DECIBEL = 'dB'
UNIT_DECIBEL_MILLIWATT = 'dBm'
UNIT_DEGREE_PER_SECOND = '°/s'
UNIT_DEGREES = '°'
UNIT_EMPTY = ''
UNIT_G = 'G'
UNIT_HECTOPASCAL = 'hPa'
UNIT_HERTZ = 'hz'
UNIT_KELVIN = 'K'
UNIT_KILOMETER = 'km'
UNIT_KILOMETER_PER_HOUR = 'km/h'
UNIT_LUX = 'lx'
UNIT_METER = 'm'
UNIT_METER_PER_SECOND_SQUARED = 'm/s²'
UNIT_MICROGRAMS_PER_CUBIC_METER = 'µg/m³'
UNIT_MICROMETER = 'µm'
UNIT_MICROSIEMENS_PER_CENTIMETER = 'µS/cm'
UNIT_MICROTESLA = 'µT'
UNIT_OHM = 'Ω'
UNIT_PARTS_PER_BILLION = 'ppb'
UNIT_PARTS_PER_MILLION = 'ppm'
UNIT_PERCENT = '%'
UNIT_PULSES_PER_MINUTE = 'pulses/min'
UNIT_SECOND = 's'
UNIT_STEPS = 'steps'
UNIT_VOLT = 'V'
UNIT_VOLT_AMPS = 'VA'
UNIT_VOLT_AMPS_REACTIVE = 'VAR'
UNIT_WATT = 'W'

DEVICE_CLASS_CONNECTIVITY = 'connectivity'
DEVICE_CLASS_MOVING = 'moving'
