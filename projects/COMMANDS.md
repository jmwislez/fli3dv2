# Fli3dv2 Commands and Valid Parameters

This document reflects the commands implemented in `lib/fli3dv2_lib/src/fli3dv2.cpp`.

## 1) Command list

| Command name | Command ID | Valid arguments | Notes |
|---|---:|---|---|
| `reboot` | `CMD_REBOOT` | `system` | Executes reboot on this subsystem, the other subsystem, or both |
| `set_opsmode` | `CMD_SET_OPSMODE` | `opsmode` | Allowed only when command is executed via TC |
| `set_parameter` | `CMD_SET_PARAMETER` | `parameter`, `value` | Generic config setter |
| `load_config` | `CMD_LOAD_CONFIG` | `bank` | Loads config from EEPROM bank |
| `save_config` | `CMD_SAVE_CONFIG` | `bank`, `tag` | Saves current config to EEPROM bank |
| `flush_fs` | `CMD_FLUSH_FS` | none | Deletes non-archive files on LittleFS |
| `list_fs` | `CMD_LIST_FS` | none | Lists files and filesystem stats |

## 2) Valid parameter values used by `reboot`

`cmd_reboot()` accepts a single byte `system` with these values:

| Constant | Meaning | Valid for |
|---|---|---|
| `SS_THIS` | Reboot this subsystem only | `reboot` |
| `SS_OTHER` | Send reboot to other subsystem | `reboot` |
| `SS_ANY` | Reboot this and send reboot to other | `reboot` |

Notes:
- `SS_THIS`, `SS_OTHER`, and `SS_ANY` are subsystem-selection constants defined in `fli3dv2.h`.
- `SS_OTHER` is intended to trigger a reboot on the peer subsystem, not the local one.

## 3) Valid parameter values used by `set_opsmode`

`cmd_set_opsmode()` accepts a single value in `opsmode`:

| Value | Name | Meaning |
|---:|---|---|
| `0` | `MODE_INIT` | init |
| `1` | `MODE_CHECKOUT` | checkout |
| `2` | `MODE_NOMINAL` | nominal |
| `3` | `MODE_MAINTENANCE` | maintenance |

These are the accepted values checked in `set_opsmode()`.

## 4) Generic `set_parameter` format

`cmd_set_parameter()` calls:

```cpp
cmd_set_parameter(tc_this->parameter, (char*)(tc_this->parameter + strlen(tc_this->parameter) + 1));
```

The payload is effectively:

```text
parameter\0value
```

Example:

```text
wifi_channel\0 6
```

---

## 5) Valid `set_parameter` names and values

### 5.1 Integer / enum parameters

| Parameter | Accepted values | Notes |
|---|---|---|
| `boot_bank` | `0..3` | Boot config bank selection |
| `wifi_channel` | `1..13` | WiFi channel |
| `target_opsmode` | `0..3` | `MODE_INIT`, `MODE_CHECKOUT`, `MODE_NOMINAL`, `MODE_MAINTENANCE` |
| `serial_baud` | `> 0` | Serial baud rate |
| `radio_baud` | `> 0` | Radio baud rate |
| `ftp_fs` | `0..3` | `FS_NONE`, `FS_LITTLEFS`, `FS_SD_MMC`, `FS_EEPROM` |
| `archive_fs` | `0..3` | same as above |
| `timestamp` | any integer | Passed to `setTime()` |
| `pressure_tm_rate` | `0..255` | |
| `motion_tm_rate` | `0..255` | |
| `gps_tm_rate` | `1`, `5`, `10`, `16` | Code checks these exact values |

### 5.2 Boolean parameters

All of the following accept `0` or `1`. Some code paths also accept `-1` as a toggle of the current value.

| Parameter | Accepted values |
|---|---|
| `wifi_ap_enable` | `0`, `1`, optionally `-1` |
| `wifi_sta_enable` | `0`, `1`, optionally `-1` |
| `ftp_enable` | `0`, `1`, optionally `-1` |
| `ota_enable` | `0`, `1`, optionally `-1` |
| `espnow_longrange` | `0`, `1`, optionally `-1` |
| `espnow_broadcast` | `0`, `1`, optionally `-1` |
| `espnow_rx_enable` | `0`, `1`, optionally `-1` |
| `espnow_tx_enable` | `0`, `1`, optionally `-1` |
| `espnow_buffer_enable` | `0`, `1`, optionally `-1` |
| `serial_rx_enable` | `0`, `1`, optionally `-1` |
| `serial_tx_enable` | `0`, `1`, optionally `-1` |
| `serial_buffer_enable` | `0`, `1`, optionally `-1` |
| `radio_rx_enable` | `0`, `1`, optionally `-1` |
| `radio_tx_enable` | `0`, `1`, optionally `-1` |
| `radio_buffer_enable` | `0`, `1`, optionally `-1` |
| `archive_enable` | `0`, `1`, optionally `-1` |
| `archive_buffer_enable` | `0`, `1`, optionally `-1` |
| `fs_enable` | `0`, `1`, optionally `-1` |
| `flush_fs_enable` | `0`, `1`, optionally `-1` |
| `sd_enable` | `0`, `1`, optionally `-1` |
| `buzzer_enable` | `0`, `1`, optionally `-1` |
| `pressure_enable` | `0` or `1` |
| `motion_enable` | `0` or `1` |
| `gps_enable` | `0` or `1` |

### 5.3 Routing parameters

Routing parameters are set using names such as:

```text
routing_espnow_<packet_name>
routing_serial_<packet_name>
routing_radio_<packet_name>
routing_archive_<packet_name>
```

Valid values:

| Parameter value | Meaning |
|---|---|
| `0` | Disable routing |
| `1` | Enable routing |
| `-1` | Toggle current setting |

The packet names must match entries in the `packet[]` table, for example:

- `tm_esp32`
- `tm_gps`
- `tm_motion`
- `tc_esp32`
- `sts_esp32`
- `cfg_esp32`
- etc.

Example:

```text
routing_espnow_tm_esp32=1
routing_serial_tc_esp32=0
routing_archive_cfg_esp32=1
```

---

## 6) EEPROM config bank arguments

For config-related commands:

| Command | Valid bank values |
|---|---|
| `load_config` | `1..3` |
| `save_config` | `1..3` |

The code checks:

```cpp
if (bank > 0 && bank < 4)
```

So bank `0` is not valid for EEPROM config operations.

---

## 7) Filesystem enum values

| Enum | Value | Meaning |
|---|---:|---|
| `FS_NONE` | `0` | none |
| `FS_LITTLEFS` | `1` | flash / LittleFS |
| `FS_SD_MMC` | `2` | SD card |
| `FS_EEPROM` | `3` | EEPROM |

---

## 8) Common usage examples

### Reboot this subsystem

```text
reboot SS_THIS
```

### Reboot the other subsystem

```text
reboot SS_OTHER
```

### Set opsmode to checkout

```text
set_opsmode 1
```

### Set WiFi channel to 6

```text
set_parameter wifi_channel 6
```

### Disable ESP-NOW TX

```text
set_parameter espnow_tx_enable 0
```

### Enable routing for TM_ESP32 over ESP-NOW

```text
set_parameter routing_espnow_tm_esp32 1
```

### Save current config to bank 2 with tag `flight`

```text
save_config 2 flight
```

### Load config from bank 1

```text
load_config 1
```

---

## 9) Notes

- `CMD_REBOOT`, `CMD_SET_OPSMODE`, and `CMD_SET_PARAMETER` are special command IDs handled before the normal checkout-only command switch.
- `CMD_SET_PARAMETER`, `CMD_LOAD_CONFIG`, `CMD_SAVE_CONFIG`, `CMD_FLUSH_FS`, and `CMD_LIST_FS` are only accepted in `MODE_CHECKOUT`.
- `CMD_REBOOT` and `CMD_SET_OPSMODE` are handled earlier and are not gated by checkout mode in the same way.

This file is based directly on the checks in `fli3dv2.cpp`.