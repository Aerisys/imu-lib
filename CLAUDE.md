# imu-lib — Drivers IMU (MPU6050 / MPU9250) avec filtres d'orientation

Bibliothèque PlatformIO / ESP-IDF qui fournit deux drivers I2C
**autoportants** pour les capteurs **MPU6050** (6 axes) et **MPU9250**
(9 axes avec magnétomètre AK8963), avec calibration, lecture
continue dans une tâche FreeRTOS dédiée, et filtres d'orientation
(**Complementary** + **Mahony**).

Utilisée par le firmware `drone` pour l'estimation d'attitude.

> Consommée depuis GitHub par le drone :
> `lib_deps = https://github.com/Aerisys/imu-lib`.
> Bumper [library.json](library.json) (actuellement `1.0.1`) à chaque
> publi puis pousser sur GitHub.

## Stack

- **Type** : library PlatformIO
- **Framework** : `espidf`
- **Plateforme** : `espressif32`
- **CPU clock** : `board_build.f_cpu = 240000000`
- **Langage** : C++

## Contenu

| Fichier | Rôle |
| ------- | ---- |
| [include/mpu9250.h](include/mpu9250.h) + [src/mpu9250.cpp](src/mpu9250.cpp) | Driver MPU9250 + AK8963. Calibration accel/gyro/mag, filtres Complementary & Mahony, `sensorTask`, healthy check, inversion d'axes, switch roll/pitch. |
| [include/mpu6050.h](include/mpu6050.h) + [src/mpu6050.cpp](src/mpu6050.cpp) | Driver MPU6050 (6 axes, sans mag). Mêmes filtres + variante `MAHONY_QUAT`. |
| [include/mpuDTO.h](include/mpuDTO.h) | DTO regroupant `accel`, `gyro`, `mag`, `orientation` + `motorSpeeds[4]` (utilisé par le drone pour télémétrie). |

## Commandes utiles

```powershell
pio run             # build standalone
pio run -t clean
```

## Comment l'utiliser (côté firmware)

```cpp
#include "mpu9250.h"

MPU9250 imu;
imu.setFilterMode(MPU9250::MAHONY);
ESP_ERROR_CHECK(imu.init(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22)); // SDA, SCL
imu.calibrate();             // ~1000 échantillons, drone immobile et à plat
imu.startSensorTask();       // lance la tâche FreeRTOS interne

auto o = imu.getOrientation();   // {roll, pitch, yaw} en degrés
auto a = imu.getAccel();         // g
auto g = imu.getGyro();          // deg/s
auto m = imu.getMag();           // µT (MPU9250 uniquement)
```

Voir aussi le bloc commenté en haut de [include/mpu9250.h](include/mpu9250.h)
pour un exemple complet.

## Constantes clés (mpu9250.h)

| Constante                    | Valeur     | Sens                                     |
| ---------------------------- | ---------- | ---------------------------------------- |
| `MPU9250_ADDR`               | `0x68`     | Adresse I2C MPU                          |
| `AK8963_ADDR`                | `0x0C`     | Adresse I2C magnéto (mode bypass)        |
| `CALIBRATION_SAMPLES`        | `1000`     | Nb d'échantillons pour calib offsets     |
| `FILTER_ALPHA`               | `0.96`     | Poids gyro dans complementary filter     |
| `MAHONY_KP` / `MAHONY_KI`    | `0.5/0.1`  | Gains du filtre Mahony                   |
| `GRAVITY`                    | `9.80665`  | m/s²                                     |

## Filtres d'orientation

- **`COMPLEMENTARY`** : combine accéléromètre (basses fréquences) et
  gyroscope intégré (hautes fréquences) via `FILTER_ALPHA`. Simple,
  rapide, suffisant pour drone en intérieur calme.
- **`MAHONY`** : approche quaternion avec feedback intégral. Plus
  robuste en dynamique forte (vibrations, virages). C'est ce qu'utilise
  le firmware drone par défaut.
- **`MAHONY_QUAT`** (MPU6050 uniquement) : variante quaternion pure.

## Conventions / pièges

- **I2C unique** : un seul `i2c_port_t` est ouvert par instance. Ne
  pas créer deux `MPU9250` sur le même bus sans gérer le mutex.
- **Calibration** : le drone **doit être immobile et à plat** pendant
  la calibration. Si tu appelles `calibrate()` en HIL (avec drone qui
  bouge), tu pollues les offsets pour rien.
- **Convention d'axes** : par défaut, la lib utilise la convention
  MPU9250 brute. Si ton montage mécanique inverse un axe ou échange
  roll/pitch, utiliser `setInvertAxis(x, y, z)` et `setSwitchRollPitch(true)`
  **avant** `init()`.
- **Magnéto AK8963** : accessible via I2C bypass du MPU9250 (`MPU9250_INT_PIN_CFG`).
  Si le mag n'est pas détecté, `magAvailable = false` et le yaw devient
  un simple intégrale gyro (drift attendu).
- **Health check** : `isSensorHealthy()` retourne `false` après un trop
  grand nombre d'`errorCount` cumulés. Utile pour failsafe drone.
- **Versioning** : à chaque modif publiée, bumper [library.json](library.json)
  et tagger sur GitHub. Sinon PlatformIO continue à servir l'ancienne.
- **Pas d'ISR** : la lecture I2C se fait en polling depuis la sensor task.
  Pas besoin d'EXT_INT du MPU pour fonctionner.

## Itération locale

Côté drone, pendant le dev :

```ini
; drone/platformio.ini
lib_deps =
    https://github.com/Aerisys/esp-lib
    file://../imu-lib
```

ou via `lib_extra_dirs = ..`.

## Tests

Pas de tests unitaires. La validation se fait sur cible (poser le
drone à plat, vérifier `getOrientation()` proche de 0/0/0, faire
des rotations contrôlées et lire les angles via les logs).
