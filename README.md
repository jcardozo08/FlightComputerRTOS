# FlightComputerRTOS

Firmware de computador de bordo para foguetes experimentais, rodando em **ESP32-S3** com **FreeRTOS**. Ele lê os sensores durante todo o voo, detecta o apogeu, aciona os eventos de recuperação (paraquedas) e transmite telemetria em tempo real para a ground station.

Este firmware sucede o [FlightCode](https://github.com/jcardozo08/FlightCode). A lógica foi reorganizada em **tasks independentes, distribuídas entre os dois núcleos** do ESP32-S3, para que a leitura de sensores e a detecção de eventos nunca sejam travadas por comunicação lenta (rádio, SD).

## Arquitetura

| Task | Responsabilidade |
|---|---|
| `TaskSensors` | Leitura de barômetro, IMU e magnetômetro, filtragem e fusão de orientação |
| `TaskFlightEvents` | Máquina de estados de voo: detecção de apogeu e acionamento dos canais de recuperação |
| `TaskCommunication` | Telemetria LoRa, GPS e registro de dados em SD/EEPROM |
| `TaskBuzzer` | Sinalização sonora de status (pronto, erro, pós-pouso para localização) |

Os dados são compartilhados entre as tasks com semáforos/mutex do FreeRTOS. Cada task é criada com `xTaskCreatePinnedToCore`, com núcleo, prioridade e stack configuráveis.

## Funcionalidades

- **Sensores:** drivers próprios para BMP280, BMP388, BMP085, MPU9250 + AK8963, ICM-20948, ADXL345, L3G4200D e HMC5883L
- **Processamento:** filtro de Kalman, filtros próprios e fusão de orientação Madgwick
- **Recuperação:** detecção de apogeu barométrica e acionamento temporizado dos canais de paraquedas
- **Telemetria:** rádios LoRa E32/E22, com troca de canal em voo por handshake com a ground station
- **GPS:** posição via TinyGPSPlus
- **Robustez:** retomada do estado de voo após reset (Preferences/NVS)
- **Simulação:** modo de simulação integrado ao [URD_APP](https://github.com/jcardozo08/URD_APP), que injeta dados de um voo real no microcontrolador para testar a lógica em solo

## Configuração

Tudo é selecionado em tempo de compilação:
- `src/lib/config.h`: sensores habilitados, intervalos de telemetria, parâmetros de LoRa, GPS e buzzer
- `src/lib/pinout.h`: pinagem de cada placa ([PlacaMainSMD_v1](https://github.com/jcardozo08/PlacaMainSMD_v1), [Essentials-V2](https://github.com/jcardozo08/Essentials-V2) etc.)

**Stack:** C++ · Arduino core para ESP32 · FreeRTOS · ESP32-S3
