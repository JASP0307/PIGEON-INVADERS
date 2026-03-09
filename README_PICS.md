# Captura Automática de Dataset — Freenove ESP32-S3 CAM

## Configuración en Arduino IDE

### 1. Instalar el paquete ESP32 de Espressif
- File > Preferences > Additional Board Manager URLs:
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
- Tools > Board > Boards Manager > buscar "esp32" > instalar versión 2.0.x o superior

### 2. Seleccionar la placa correcta
```
Tools > Board     : "ESP32S3 Dev Module"
Tools > Flash Size: "16MB (128Mb)"
Tools > PSRAM     : "OPI PSRAM"
Tools > Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
Tools > USB Mode  : "Hardware CDC and JTAG"
Tools > Upload Speed: 921600
```

### 3. No se requieren librerías externas
Todo usa librerías incluidas en el paquete ESP32 de Espressif:
- esp_camera.h
- SD_MMC.h
- FS.h

---

## Preparar la tarjeta SD
- Formato: **FAT32**
- Tamaño recomendado: 16GB o 32GB
- La carpeta `/dataset` se crea automáticamente

---

## Parámetros configurables en el código

| Parámetro           | Descripción                              | Valor por defecto |
|---------------------|------------------------------------------|-------------------|
| `MODO_CAPTURA`      | 1=intervalo fijo, 2=movimiento           | 1                 |
| `INTERVALO_MS`      | Tiempo entre capturas (ms)              | 10000 (10 seg)    |
| `UMBRAL_MOVIMIENTO` | Sensibilidad movimiento (0-255)         | 30                |
| `COOLDOWN_MOVIMIENTO`| Tiempo mínimo entre capturas modo 2   | 3000 (3 seg)      |
| `RESOLUCION`        | Resolución de captura                   | FRAMESIZE_QVGA    |
| `CALIDAD_JPEG`      | Calidad JPEG (0-63, menor=mejor)        | 10                |

---

## Estrategia de captura sugerida para el dataset

### Fase 1 — Plataforma vacía (clase negativa)
- Modo 1, intervalo 30 segundos
- Capturar durante 2-3 horas en distintos horarios
- Meta: ~80-100 imágenes

### Fase 2 — Con palomas (clase positiva)
- Modo 2 (detección de movimiento) para no perder llegadas
- Dejar el dispositivo funcionando durante el día
- Meta: ~150-200 imágenes

### Fase 3 — Condiciones difíciles
- Cambiar wb_mode en el código según la hora:
  - Soleado: s->set_wb_mode(s, 1)
  - Nublado : s->set_wb_mode(s, 2)
- Capturar en contraluz (mañana/atardecer)

---

## Indicadores LED

| Patrón               | Significado                    |
|----------------------|-------------------------------|
| 3 parpadeos rápidos  | Sistema listo                 |
| 1 parpadeo corto     | Imagen guardada correctamente |
| 10 parpadeos rápidos | Error de cámara               |
| 5 parpadeos lentos   | Error de tarjeta SD           |

---

## Monitoreo por Serial (115200 baud)

Conectar por USB y abrir el Monitor Serial para ver:
- Confirmación de inicio de cámara y SD
- Ruta de cada imagen guardada
- Estadísticas cada 10 imágenes (total, espacio libre)
- Porcentaje de movimiento detectado (modo 2)

---

## Solución de problemas

**"No se pudo inicializar la cámara"**
→ Verificar que se seleccionó "OPI PSRAM" en Tools
→ Intentar con un cable USB diferente (algunos son solo carga)

**"No se pudo montar la tarjeta SD"**
→ Formatear la SD en FAT32 (no exFAT)
→ En Windows usar "SD Card Formatter" (herramienta oficial SD Association)
→ Probar con otra tarjeta SD (algunas marcas tienen incompatibilidades)

**Imágenes muy oscuras o sobreexpuestas**
→ Ajustar s->set_ae_level(s, valor) en el código (-2 a +2)
→ Dar 5-10 segundos después del encendido antes de la primera captura

**La imagen sale espejada**
→ Cambiar s->set_hmirror(s, 1) o s->set_vflip(s, 1) según corresponda
