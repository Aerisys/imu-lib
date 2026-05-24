# imu-lib

> Lib IMU pour ESP32 (ESP-IDF / PlatformIO) — MPU6050 + MPU9250 (avec AK8963), pensée pour le **contrôle de drone**.

Drivers I²C, calibration persistée NVS, filtre Mahony 9-DOF avec quaternion, lecture pilotée par interrupt `DATA_READY` jusqu'à **1 kHz**, snapshot atomique lock-free pour la boucle PID, notch filter configurable contre les vibrations moteur.

---

## Highlights

| Feature | Détail |
|---|---|
| **Bus I²C** | Driver ESP-IDF v5 (`i2c_master.h`). Bus **possédé par le consommateur** — la lib s'attache comme device. Plusieurs capteurs peuvent partager le bus. |
| **Cadence** | 1 kHz INT-driven (broche `DATA_READY` du MPU + GPIO ISR). Fallback polling 100 Hz si pas d'INT câblé. |
| **Vitesse I²C** | 400 kHz Fast Mode par défaut. Budget mesuré : **612 µs I²C + 69 µs math = 681 µs / boucle** ⇒ ~32 % de marge à 1 kHz. |
| **Fusion 9-DOF** | Mahony AHRS avec correction magnétomètre — yaw borné, drift < 0.1°/min mesuré. Quaternion exposé pour PID sans gimbal lock. |
| **Calibration** | `calibrateGyroAccel()` (10 s immobile, auto-NVS) et `calibrateMag()` (30 s figure-8, séparé). Compensation T° gyro configurable. |
| **Synchro consommateur** | `waitForNewSample()` (binary sem) + `getSnapshot()` (seqlock atomique) → PID drone **timé par le capteur**, plus de phase drift. |
| **Notch filter** | Biquad IIR par axe gyro configurable runtime pour supprimer la fondamentale moteur (vibrations). |
| **Architecture** | Interface abstraite `IMUSensor` → MPU6050 / MPU9250 / ICM-* futurs interchangeables polymorphiquement. |
| **Persistance** | Offsets accel/gyro/mag + coeff T° dans NVS (blob versionné). Skip de la calibration à chaque boot. |

---

## Câblage MPU9250 → ESP32 (typique)

| MPU9250 / GY-9250 | ESP32 | Rôle |
|---|---|---|
| `VCC` | `3V3` | Alim (5 V OK si module avec LDO onboard, type GY-91) |
| `GND` | `GND` | Masse |
| `SDA` | `GPIO 21` | I²C Data |
| `SCL` | `GPIO 22` | I²C Clock |
| `INT` | `GPIO 19` | DATA_READY (active pour 1 kHz) |
| `AD0` | NC ou GND | Adresse 0x68 |

> Pour Fast Mode 400 kHz : les pull-ups internes ESP32 (~45 kΩ) suffisent en bench. Sur drone (traces longues, parasites) prévois des pull-ups externes 2.2 kΩ sur SDA/SCL.

---

## Quick start (exemple standalone)

Voir [`examples/basic/main.cpp`](examples/basic/main.cpp). En résumé :

```cpp
#include <mpu9250.h>
#include "imu_sensor.h"
#include "driver/i2c_master.h"
#include "nvs_flash.h"

extern "C" void app_main()
{
    // 1. NVS (pour la persistance calibration)
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // 2. I²C bus (owned by the app — pas par la lib)
    i2c_master_bus_config_t busCfg = {};
    busCfg.i2c_port           = I2C_NUM_0;
    busCfg.sda_io_num         = GPIO_NUM_21;
    busCfg.scl_io_num         = GPIO_NUM_22;
    busCfg.clk_source         = I2C_CLK_SRC_DEFAULT;
    busCfg.glitch_ignore_cnt  = 7;
    busCfg.flags.enable_internal_pullup = true;
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&busCfg, &bus));

    // 3. IMU
    static MPU9250 imu;
    MPU9250::Config cfg;
    cfg.intPin = GPIO_NUM_19;          // active 1 kHz INT-driven
    ESP_ERROR_CHECK(imu.init(bus, cfg));

    if (imu.getCalibrationStatus() != MPU9250::CALIBRATED)
        imu.calibrateGyroAccel();      // 10 s immobile, async, auto-persistée

    imu.setFilterMode(MPU9250::MAHONY);
    imu.setMahonyGains(1.0f, 0.0f);    // Ki=0 pour drone (évite windup en manœuvre)
    imu.startSensorTask();

    // 4. Boucle PID (cadence du capteur)
    IMUSensor* sensor = &imu;
    while (true) {
        if (sensor->waitForNewSample(100) != ESP_OK) continue;
        IMUSensor::SampleBundle s = sensor->getSnapshot();
        // ... PID drone : s.quaternion, s.gyro, s.timestampUs ...
    }
}
```

---

## Intégration dans un projet drone

### `platformio.ini` du projet drone

```ini
[env:drone]
platform = espressif32
board    = esp32dev
framework = espidf
monitor_speed = 115200

lib_deps =
    https://github.com/Aerisys/imu-lib

build_flags =
    ; Optionnel : profiler I²C/math sur la sensor task pour debug timing
    ; -DMPU9250_PROFILER=1
```

> **Recommandé** : passer `CONFIG_FREERTOS_HZ=1000` dans `sdkconfig` pour exploiter pleinement le 1 kHz INT-driven (les wakeups ISR fonctionnent à tout tick rate, mais les timeouts watchdog s'arrondissent à 10 ms en default 100 Hz).

### Itération locale (dev simultané lib + drone)

```ini
lib_deps =
    file://../imu-lib
```

ou `lib_extra_dirs = ..`.

### Pattern d'usage côté firmware drone

```cpp
// Le projet drone fournit l'I²C bus à la lib — un seul bus partagé baro + IMU + ...
class DroneFirmware {
    i2c_master_bus_handle_t bus;
    MPU9250 imu;
    // ... baro, ESC, ...

    void boot() {
        // I²C unique pour tous les périphériques
        i2c_new_master_bus(&busCfg, &bus);
        bmp280.attach(bus);   // exemple
        imu.init(bus, imuCfg);
        // ...
    }
};

// Boucle de contrôle séparée
void controlTask(void* arg) {
    IMUSensor* imu = ...;   // injection polymorphique
    while (1) {
        imu->waitForNewSample(20);
        auto s = imu->getSnapshot();
        // Quaternion-based PID — pas de gimbal lock en flip/loop
        pid_update(s.quaternion, s.gyro, s.timestampUs);
        actuate_motors();
    }
}
```

### Architecture FreeRTOS recommandée (drone ESP32 dual-core)

La lib crée sa propre task `mpu9250_task` au sein de `startSensorTask()` — pas de polling à faire toi-même. Configure son **core** et sa **priorité** via `Config::taskCoreId` / `taskPriority` selon ton scénario :

| Tâche | Priorité | Core | Notes |
|---|---|---|---|
| **IMU sensor** (cette lib) | `5` (default) | `APP_CPU_NUM` | `cfg.taskCoreId = APP_CPU_NUM;` |
| **PID / control** (firmware) | `4` (= IMU – 1) | `APP_CPU_NUM` | sync via `waitForNewSample()`, **lockstep** avec l'IMU |
| **Wi-Fi / lwIP** (ESP-IDF) | défaut | `PRO_CPU_NUM` | via `sdkconfig` : `CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0=y`, `CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0=y` |
| **Telemetry / MQTT / log** | `2` | `PRO_CPU_NUM` | basse cadence, accède `getSnapshot()` à la demande |

**Principe** : core 1 (APP_CPU) = temps réel deterministe, core 0 (PRO_CPU) = networking. Sans cette séparation, les ISR Wi-Fi peuvent injecter ~50 µs de jitter dans la boucle PID.

```cpp
MPU9250::Config cfg;
cfg.intPin       = GPIO_NUM_19;
cfg.taskCoreId   = APP_CPU_NUM;   // pinne la sensor task sur le core "temps réel"
cfg.taskPriority = 5;              // ton PID doit être à 4
imu.init(bus, cfg);
```

> Pour le bench standalone sans Wi-Fi : laisse `tskNO_AFFINITY` (default). Le pinning ne devient utile que dès que des stacks lourds (Wi-Fi/BLE) entrent en jeu.

---

## API publique (résumé)

### Interface commune `IMUSensor`

```cpp
class IMUSensor {
    // Types
    struct Vector3       { float x, y, z; };
    struct Orientation   { float roll, pitch, yaw; };
    struct Quaternion    { float w, x, y, z; };
    struct SampleBundle  { Vector3 accel, gyro, mag;
                           Orientation orientation;
                           Quaternion quaternion;
                           float temperature;
                           uint64_t timestampUs; };
    enum   CalibrationStatus { NOT_CALIBRATED, CALIBRATING, CALIBRATED };
    enum   FilterMode    { COMPLEMENTARY, MAHONY };

    // Lifecycle
    esp_err_t calibrate();              // alias de calibrateGyroAccel
    esp_err_t startSensorTask();
    CalibrationStatus getCalibrationStatus();

    // Config
    esp_err_t setFilterMode(FilterMode);
    esp_err_t setInvertAxis(bool x, bool y, bool z);
    esp_err_t setSwitchRollPitch(bool);

    // Data — lock-free (seqlock) sur MPU9250
    Vector3      getAccel();
    Vector3      getGyro();
    Vector3      getMag();           // {0,0,0} si pas de mag
    Orientation  getOrientation();
    Quaternion   getQuaternion();    // identité si pas de filtre quaternion
    float        getTemperature();
    bool         isSensorHealthy();

    // Snapshot atomique + sync
    SampleBundle getSnapshot();
    esp_err_t    waitForNewSample(uint32_t timeoutMs); // 0 = bloque indéfiniment
};
```

### Extensions MPU9250-only

```cpp
class MPU9250 : public IMUSensor {
    // Init (Config sensor-spécifique)
    struct Config {
        uint8_t     mpuAddr       = 0x68;
        uint8_t     magAddr       = 0x0C;
        uint32_t    sclSpeedHz    = 400000;
        gpio_num_t  intPin        = GPIO_NUM_NC;     // GPIO_NUM_NC = polling fallback
        BaseType_t  taskCoreId    = tskNO_AFFINITY;  // ou APP_CPU_NUM (1) / PRO_CPU_NUM (0)
        UBaseType_t taskPriority  = 5;               // FreeRTOS prio de la sensor task
        uint32_t    taskStackSize = 4096;            // stack en octets (3072 mini avec profiler)
    };
    esp_err_t init(i2c_master_bus_handle_t bus, const Config& = {});

    // Calibration séparée
    esp_err_t calibrateGyroAccel();          // 10 s immobile
    esp_err_t calibrateMag();                // 30 s figure-8

    // Tuning runtime
    esp_err_t setMahonyGains(float kp, float ki);
    esp_err_t setGyroTempCompCoeff(Vector3 coeff);  // deg/s par degC
    esp_err_t setGyroNotch(float centerHz, float bandwidthHz, float sampleRateHz = 1000);
    esp_err_t clearGyroNotch();

    // Persistance NVS (auto au boot + auto en fin de calib)
    esp_err_t saveCalibration();
    esp_err_t loadCalibration();
    esp_err_t clearStoredCalibration();
};
```

---

## Pièges connus / bonnes pratiques drone

- **PID en quaternion, pas Euler**. À pitch ≈ ±90° (flip, looping) les angles Euler dégénèrent (gimbal lock). Le quaternion reste continu.
- **`Ki` Mahony à 0 par défaut**. Un Ki non-nul provoque du windup intégral sur manœuvres agressives. À activer seulement si dérive gyro mesurée en vol.
- **Calibration mag NON automatique au boot**. Elle exige 30 s de rotation utilisateur — à déclencher explicitement (bouton de boot, mode config, etc.).
- **NVS** doit être init par le projet drone (`nvs_flash_init()`) ; la lib s'y attache passivement.
- **I²C bus** owned by app. Si plusieurs capteurs sur le bus, créer le bus **une fois** au boot et le passer à chaque driver.
- **Pull-ups 400 kHz** : modules GY-91 / GY-9250 ont 4.7 kΩ on-board, OK. Bare MPU9250 → ajouter 2.2 kΩ externes.
- **Notch filter** : nécessite de mesurer la fondamentale moteur (FFT du gyro en hover) avant de pouvoir le tuner. Inutile / contre-productif si appliqué au hasard.

---

## Profiler & debug

Ajoute `-DMPU9250_PROFILER=1` dans `build_flags` pour activer les logs de timing dans la sensor task :

```
I (xxx) PROFILER: --- Profiling sur 500 boucles ---
I (xxx) PROFILER: I2C  (Moy)  : 612 us
I (xxx) PROFILER: I2C  (Max)  : 1102 us
I (xxx) PROFILER: Math (Moy)  : 69 us
```

À désactiver en prod.

---

## Versioning & publication

- [`library.json`](library.json) → bump à chaque publi (`major.minor.patch`)
- Tag git en cohérence
- PlatformIO sert la dernière version satisfaisant la contrainte du consommateur

---

## Tests sur cible

Procédure complète détaillée dans [`docs/AGENTS.md`](docs/AGENTS.md#tests-sur-cible). Résumé :

1. Smoke : `WHO_AM_I` MPU + AK8963
2. NVS : reboot après calibration → skip
3. Quaternion au repos : stable, pas de drift
4. Tilt : roll/pitch suivent les mouvements
5. Yaw stable : drift < 0.1°/min mesuré
6. Rate : profiler → 500 boucles toutes les ~510 ms (≈ 1 kHz)

Pas de tests unitaires automatisés actuellement (le répertoire `test/` est vide). Contribution bienvenue.

---

## Licence

Voir [`LICENCE`](LICENCE).
