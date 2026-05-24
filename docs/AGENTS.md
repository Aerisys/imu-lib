# imu-lib — Contexte technique pour agents IA

> Refresh majeur v1.1.0 — refonte architecture, perf, et correctness.
> Le diff vs. v1.0 est massif : **ne pas se fier aux conventions antérieures**.

Bibliothèque PlatformIO / ESP-IDF qui fournit deux drivers I²C pour les capteurs **MPU6050** (6 axes) et **MPU9250** (9 axes avec magnétomètre AK8963), unifiés derrière une interface abstraite `IMUSensor`. Pensée pour le **contrôle de drone** : pilotage par interrupt `DATA_READY` (1 kHz), calibration persistée NVS, fusion Mahony 9-DOF avec quaternion, snapshot atomique lock-free, notch filter contre vibrations moteur.

Utilisée par le firmware `drone` pour l'estimation d'attitude.

> Consommée depuis GitHub par le drone :
> `lib_deps = https://github.com/Aerisys/imu-lib`.
> Bumper [`library.json`](../library.json) (actuellement `1.1.0`) à chaque publi puis pousser sur GitHub.

## Stack

- **Type** : library PlatformIO
- **Framework** : `espidf` (ESP-IDF v5+ requis — utilise `driver/i2c_master.h` qui remplace l'ancien `driver/i2c.h` deprecated)
- **Plateforme** : `espressif32`
- **CPU clock** : `board_build.f_cpu = 240000000`
- **Langage** : C++17 (utilise `<atomic>`, `std::memory_order_*`)
- **Recommandé** : `CONFIG_FREERTOS_HZ=1000` dans le `sdkconfig` du consommateur pour exploiter le 1 kHz INT-driven

## Architecture

```
IMUSensor (interface pure virtual)
   ├── MPU9250  (full impl : 9-DOF, NVS, notch, T° comp, snapshot atomique)
   └── MPU6050  (impl minimale : 6-DOF, stubs pour les méthodes interface qu'il ne supporte pas)
```

- **Interface** [`include/imu_sensor.h`](../include/imu_sensor.h) : types communs (`Vector3`, `Orientation`, `Quaternion`, `SampleBundle`), enums (`CalibrationStatus`, `FilterMode`), méthodes virtuelles minimum (calibrate, getters, getSnapshot, waitForNewSample).
- Le consommateur drone peut utiliser `IMUSensor*` polymorphiquement dans son control loop ; init et tuning avancé restent sur le type concret.

## Contenu

| Fichier | Rôle |
| ------- | ---- |
| [`include/imu_sensor.h`](../include/imu_sensor.h) | **Interface abstraite** commune. Types `Vector3/Orientation/Quaternion/SampleBundle`, enums `CalibrationStatus/FilterMode`, méthodes virtuelles. Defaults pour `getMag` (zéro) et `getQuaternion` (identité) afin que les capteurs sans mag/sans filtre quat puissent l'implémenter sans surcharger. |
| [`include/mpu9250.h`](../include/mpu9250.h) + [`src/mpu9250.cpp`](../src/mpu9250.cpp) | Driver MPU9250 + AK8963 (mag accédé en I²C bypass mode, device séparé sur le bus). Mahony 9-DOF, calibration split gyro/accel + mag, persistance NVS, compensation T° gyro, notch biquad par axe, INT DATA_READY 1 kHz, snapshot seqlock, binary sem `waitForNewSample`. |
| [`include/mpu6050.h`](../include/mpu6050.h) + [`src/mpu6050.cpp`](../src/mpu6050.cpp) | Driver MPU6050 (6 axes, sans mag). Filtre Complementary + Mahony. Implémente l'interface `IMUSensor` mais avec des stubs no-op pour `setInvertAxis` / `setSwitchRollPitch`. **Pas** au niveau de feature du MPU9250 (pas de NVS, pas de seqlock, pas de notch) — porter au besoin. |
| [`include/mpuDTO.h`](../include/mpuDTO.h) | DTO IMU-only. Dépend de `IMUSensor` (pas du type concret). **Ne contient PAS de motor stuff** (anti-leak d'abstraction). |
| [`examples/basic/main.cpp`](../examples/basic/main.cpp) | Exemple standalone — **pas compilé** quand la lib est consommée comme dépendance. Démontre init bus + IMU + waitForNewSample + getSnapshot. |
| [`src/CMakeLists.txt`](../src/CMakeLists.txt) | ESP-IDF component registration. `REQUIRES driver nvs_flash esp_timer`. Inclut conditionnellement `examples/basic/main.cpp` pour la build standalone. |
| [`library.json`](../library.json) | Manifest PlatformIO. `export.include` / `exclude` explicites — `examples/`, `test/`, etc. exclus quand la lib est tirée comme dep. |

## Commandes utiles

```powershell
pio run             # build standalone (avec examples/basic/main.cpp)
pio run -t upload   # flash
pio device monitor  # 115200 baud
pio run -t clean

# Profiler I²C/math dans la sensor task
pio run -t upload --project-option="build_flags=-DMPU9250_PROFILER=1"
```

## Comment l'utiliser (côté firmware drone)

### Setup minimal

```cpp
#include "mpu9250.h"
#include "imu_sensor.h"
#include "driver/i2c_master.h"
#include "nvs_flash.h"

extern "C" void app_main()
{
    // 1. NVS (pour persistance calibration)
    nvs_flash_init();   // gérer ESP_ERR_NVS_NO_FREE_PAGES en prod

    // 2. Bus I²C — owned by l'app, partageable avec d'autres capteurs
    i2c_master_bus_config_t busCfg = {};
    busCfg.i2c_port           = I2C_NUM_0;
    busCfg.sda_io_num         = GPIO_NUM_21;
    busCfg.scl_io_num         = GPIO_NUM_22;
    busCfg.clk_source         = I2C_CLK_SRC_DEFAULT;
    busCfg.glitch_ignore_cnt  = 7;
    busCfg.flags.enable_internal_pullup = true;
    i2c_master_bus_handle_t bus;
    i2c_new_master_bus(&busCfg, &bus);

    // 3. IMU
    static MPU9250 imu;
    MPU9250::Config cfg;
    cfg.intPin        = GPIO_NUM_19;    // 1 kHz INT-driven (GPIO_NUM_NC = polling 100 Hz)
    cfg.taskCoreId    = APP_CPU_NUM;    // optionnel : core 1 = temps réel (sinon tskNO_AFFINITY)
    cfg.taskPriority  = 5;              // optionnel : default 5. Le PID drone doit être à 4.
    cfg.taskStackSize = 4096;           // optionnel : default 4096. Mini 3072 avec profiler.
    imu.init(bus, cfg);

    if (imu.getCalibrationStatus() != MPU9250::CALIBRATED)
        imu.calibrateGyroAccel();   // 10 s immobile, auto-persiste NVS

    imu.setFilterMode(MPU9250::MAHONY);
    imu.setMahonyGains(1.0f, 0.0f); // Ki=0 sécurité drone (anti-windup)
    imu.startSensorTask();

    // 4. Boucle PID en lockstep avec le capteur
    IMUSensor* sensor = &imu;
    while (true) {
        sensor->waitForNewSample(100);
        auto s = sensor->getSnapshot();
        // PID quaternion-based avec s.quaternion, s.gyro
    }
}
```

### Calibration mag (séparée, explicite)

```cpp
// À déclencher depuis un mode config / bouton de boot — PAS au boot normal
imu.calibrateMag();   // 30 s de rotation figure-8 demandée à l'utilisateur
```

## Constantes clés ([`include/mpu9250.h`](../include/mpu9250.h))

| Constante                    | Valeur     | Sens                                     |
| ---------------------------- | ---------- | ---------------------------------------- |
| `MPU9250_ADDR`               | `0x68`     | Adresse I²C MPU (default, `Config::mpuAddr`) |
| `AK8963_ADDR`                | `0x0C`     | Adresse I²C magnéto (bypass mode)        |
| `CALIBRATION_SAMPLES`        | `1000`     | 10 s à 100 Hz, gyro/accel immobile       |
| `MAG_CALIBRATION_SAMPLES`    | `3000`     | 30 s à 100 Hz, mag rotation              |
| `FILTER_ALPHA`               | `0.96`     | Poids gyro dans Complementary            |
| `MAHONY_KP` / `MAHONY_KI`    | `0.5 / 0.0`| Default Mahony. **Ki=0** par sécurité drone (anti-windup). Tunable runtime via `setMahonyGains()`. |
| `GRAVITY`                    | `9.80665`  | m/s²                                     |
| `MPU9250_I2C_TIMEOUT_MS`     | `100`      | Timeout `i2c_master_transmit*`           |
| `MPU9250_NVS_NAMESPACE`      | `"imu_lib"`| Namespace NVS                            |
| `MPU9250_CALIB_VERSION`      | `1`        | Version du blob calibration — bump si changement de layout `CalibBlob` |

## Filtres d'orientation

- **`COMPLEMENTARY`** : combine accéléromètre (basses fréquences) et gyroscope intégré (hautes fréquences) via `FILTER_ALPHA`. Simple, rapide, mais yaw uniquement par intégration gyro (drift attendu).
- **`MAHONY`** : **fusion 9-DOF** complète (accel + gyro + mag) en quaternion avec feedback proportionnel + intégral. Yaw borné au nord magnétique. C'est ce qu'utilise le firmware drone par défaut. Drift mesuré < 0.1°/min en bench.

## Architecture interne — points clés à comprendre

### Sensor task (priorité, stack, core tous configurables via `Config`)

```
boucle :
  attendre INT DATA_READY (ulTaskNotifyTake, timeout 20 ms)
  burst I²C 14 octets (accel + temp + gyro) — toujours
  conditionnellement (every 9 ms via nextMagReadUs) : ST1 mag + payload mag 7 octets
  appliquer offsets + invertAxis + compensation T° gyro
  appliquer notch biquad si gyroNotchEnabled
  prendre dataMutex (mutations internes uniquement)
    sensor->accel/gyro/mag/temp <- locales
    processMeasurements(dt)  -> update orientation, q, gyroIntegrated
  relâcher mutex
  --- publication ---
  seqlock writer protocol :
    bundleSeq++ (odd : "in progress")
    publishedBundle <- snapshot des membres
    bundleSeq++ (even : "stable")
  xSemaphoreGive(sampleSem)
```

### Consumer (PID drone, n'importe quelle task)

```
boucle :
  waitForNewSample(timeoutMs)  -> xSemaphoreTake(sampleSem, ...)
  s = getSnapshot()             -> seqlock read (acquire), lock-free
  // utiliser s.quaternion, s.gyro, s.timestampUs pour le PID
```

### Synchro

- **Pas un seul mutex** pris sur le chemin reader (getters / getSnapshot). Mutex `dataMutex` uniquement pour mutations internes (calibration, setters).
- **Seqlock 1-writer / N-readers** sur `publishedBundle` (`std::atomic<uint32_t> bundleSeq` impair = writing, pair = stable).
- **Binary semaphore** `sampleSem` pour la notification ; binary = jamais de backlog (consumer toujours sur le dernier sample).

## Conventions / pièges (lus *avant* de modifier)

- **I²C bus owned by app, pas par la lib**. La lib appelle `i2c_master_bus_add_device` / `i2c_master_bus_rm_device` ; **jamais** `i2c_new_master_bus` ni `i2c_del_master_bus`. Le destructeur retire les devices mais pas le bus.
- **NVS pas init par la lib**. Le consommateur doit appeler `nvs_flash_init()`. Si pas init, `loadCalibration` log un warning et retourne, le reste de l'init continue.
- **`INT_PIN_CFG` = `0x12`** (LATCH=0, ANYRD=1, BYPASS=1) — **PAS `0x22`** comme dans la v1.0 qui était un bug latent (LATCH=1 → INT latche haut → ISR fire une seule fois). Changement crucial validé par profiler.
- **Throttle mag à 9 ms** (`nextMagReadUs` local à `sensorTask`). L'AK8963 tourne à 100 Hz, polling à 1 kHz gaspille 390 µs/itér en stale data → la boucle ne tiendrait pas le budget 1 ms.
- **Quaternion exposé**. Le PID drone *doit* l'utiliser plutôt que `getOrientation()` pour éviter gimbal lock à ±90° pitch (flips, loops, vol inversé). `getOrientation()` reste exposé pour debug humain.
- **`MAHONY_KI = 0` par défaut**. Le `0.1` de la v1.0 causait du windup intégral sur manœuvres agressives drone (bias latché après chaque flip). Réactivable runtime via `setMahonyGains(kp, ki)` si dérive gyro mesurée en vol nécessite Ki > 0.
- **`switchRollPitch` initialisé `false`** dans le constructeur (était indéterminé → UB en v1.0).
- **Sensor task pinning** : la lib utilise `xTaskCreatePinnedToCore` avec `Config::taskCoreId` (default `tskNO_AFFINITY` = scheduler libre). Pour drone avec Wi-Fi actif, pinner sur `APP_CPU_NUM` ET pinner le stack IDF networking sur `PRO_CPU_NUM` via `sdkconfig` (`CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0=y`, `CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0=y`). Sinon ~50 µs de jitter occasionnel sur l'IMU.
- **Priorité IMU vs PID** : le PID du consommateur doit être à `taskPriority - 1` (default IMU=5, donc PID=4). Si égaux, la sensor task peut être préemptée par le PID juste après une publication — pas catastrophique grâce au seqlock, mais ajoute des retries inutiles côté reader.
- **Stack sizing** : `Config::taskStackSize` default 4096 octets. Peak observé ~1.5 kB sur Mahony 9-DOF + ESP_LOG buffers. Si tu actives `MPU9250_PROFILER=1`, **ne descends pas sous 3072**. Pour mesurer en vol, `uxTaskGetStackHighWaterMark(taskHandle)` retourne le minimum jamais atteint.
- **Calibration mag NON automatique**. `calibrate()` (interface) = `calibrateGyroAccel()` uniquement. Mag = `calibrateMag()` séparé, explicite, 30 s de figure-8.
- **Persistance NVS auto** : `init()` appelle `loadCalibration()`, calib réussie appelle `saveCalibration()`. Le consommateur n'a pas besoin de gérer le storage.
- **Versioning blob NVS** (`MPU9250_CALIB_VERSION`) : si on modifie le layout `CalibBlob` (anonyme dans `mpu9250.cpp`), **incrémenter** sinon les anciens blobs seront lus comme valides et corromperont les offsets. Le `static_assert` sur `sizeof(CalibBlob)` attrape les changements silencieux.
- **MPU6050 incomplet** : implémente l'interface mais sans NVS, notch, snapshot atomique, etc. Stubs pour les méthodes qu'il ne supporte pas (return `ESP_OK` + WARN). Porter au besoin si un projet l'utilise sérieusement.
- **Pull-ups I²C** : 400 kHz Fast Mode requiert ~2.2 kΩ. Pull-ups internes ESP32 (~45 kΩ) marchent en bench, **insuffisants sur drone** (traces longues + EMI). Modules GY-91 / GY-9250 ont 4.7 kΩ on-board → OK.
- **Pas de tests unitaires** dans `test/` actuellement. Validation = procédure sur cible (voir plus bas).

## Profiler

Activer avec `-DMPU9250_PROFILER=1` dans les `build_flags`. Logs toutes les 500 boucles :

```
I (xxx) PROFILER: --- Profiling sur 500 boucles ---
I (xxx) PROFILER: I2C  (Moy)  : 612 us
I (xxx) PROFILER: I2C  (Max)  : 1102 us
I (xxx) PROFILER: Math (Moy)  : 69 us
```

Valeurs **validées en bench** (MPU9250 + AK8963, GY-9250 module, ESP32 DevKitC, I²C 400 kHz, pull-ups module 4.7 kΩ).

| Métrique | Valeur observée | Cible 1 kHz |
|---|---|---|
| I²C moyen | 612 µs | < 1000 µs ✓ |
| I²C max (rare) | 1100 µs | dépassement ~3 % des itérs, resync auto |
| Math (Mahony 9-DOF) | 69 µs | — |
| Cadence effective | ~980 Hz | objectif 1 kHz |

## Itération locale

Côté drone, pendant le dev :

```ini
; drone/platformio.ini
lib_deps =
    https://github.com/Aerisys/esp-lib
    file://../imu-lib
```

ou via `lib_extra_dirs = ..`.

## Tests sur cible

Procédure complète pour valider la lib sur hardware physique (MPU9250 + ESP32).

### Câblage

| MPU9250 | ESP32 |
|---|---|
| `VCC` | `3V3` |
| `GND` | `GND` |
| `SDA` | `GPIO 21` |
| `SCL` | `GPIO 22` |
| `INT` | `GPIO 19` |
| `AD0` | NC ou GND (adr 0x68) |

### Séquence de tests

1. **Smoke test** : boot → logs `AK8963 magnetometer found`, `DATA_READY interrupt enabled on GPIO 19`, `Sensor task started`. Si `WHO_AM_I` fail → câblage / pull-ups / module défectueux.
2. **Calibration + NVS persistance** : poser immobile 10 s → logs `Gyro/Accel calibration progress: ...` puis `Calibration saved to NVS`. Reboot → `Loaded calibration from NVS — skipping cold calibration`.
3. **Quaternion au repos** : log `getQuaternion()` pendant 30 s → composantes stables (< 0.01 oscillation), gyro ≈ 0.
4. **Tilt** : incliner physiquement → `roll`/`pitch` suivent. Au-delà de ±90° : gimbal lock Euler attendu, quaternion continue correctement.
5. **Yaw stable** : laisser immobile 2-3 min → drift < 0.5° (validation fusion mag).
6. **Yaw rotation + retour** : tourner 360° puis revenir à position physique → écart < 5° (sans calib mag) ou < 2° (avec).
7. **Cadence** : activer `-DMPU9250_PROFILER=1`, vérifier profiler log toutes les ~510 ms (= 500 boucles à 1 kHz).
8. **Mag calibration** (optionnel) : appeler `calibrateMag()`, faire figure-8 30 s → écart au retour position < 2°.

### Résultats validés (bench actuel)

| Test | Critère | Résultat |
|---|---|---|
| Smoke | WHO_AM_I MPU + AK8963 | ✅ |
| Calib + NVS | reboot skip calib | ✅ |
| Yaw stable | drift < 0.5°/min | ✅ < 0.1°/min |
| Cadence INT | ~1 kHz effectif | ✅ ~980 Hz |
| I²C 400 kHz | budget < 1 ms | ✅ 681 µs moyen |
| Mahony convergence | settle < 2 s | ✅ ~1.5 s à Kp=1.0 |

## Évolutions à anticiper

- **MPU6050 catch-up** : porter NVS, snapshot atomique, T° comp, notch quand un projet l'utilise sérieusement
- **Tests unitaires** : `test/` vide, idéal : seqlock writer/reader, biquad notch (impulse + freq response), NVS round-trip
- **MPU9250 EOL** : envisager ICM-42688-P comme successeur (SPI uniquement, mais 32 kHz gyro, FIFO 2K) — facile à brancher via l'interface `IMUSensor`
- **Dynamic notch** : recalcul des coeffs en vol à partir de la fondamentale moteur (FFT live ou télémétrie ESC)
