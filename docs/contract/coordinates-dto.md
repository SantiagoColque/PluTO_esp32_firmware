# Contrato de coordenadas: `device/<device_id>/coordinates/polar`

Formato del payload que el servidor PluTO publica para que el ESP32 oriente la antena.
Este documento es la única fuente de verdad del formato: el decoder en C
(`PluTO_esp32_firmware`) y el encoder en Python (`PluTO`) se implementan por separado
contra lo que dice acá.

Estado: **v1**, formato acordado, sin implementar en ninguno de los dos lados.

## 1. Propósito y alcance

El servidor calcula una trayectoria azimut/elevación y se la entrega a la placa por
adelantado. Ambos extremos tienen el reloj sincronizado por NTP, de modo que cada punto
puede llevar el **instante en que la antena debe estar apuntando ahí**, en vez de
ejecutarse apenas llega el mensaje.

Las referencias al SRS (HU-nn, RF, RNF, RA) que aparecen abajo son de **coherencia**, no
de obligación: el SRS es un documento propio del proyecto y puede cambiar. Cada decisión
de formato se justifica por sí sola; donde un requisito es lo único que fija un número,
está dicho explícitamente.

## 2. Tópico, QoS y `retain`

| | |
|---|---|
| tópico | `device/<device_id>/coordinates/polar` |
| dirección | servidor → ESP32 |
| `device_id` | MAC de la interfaz station, doce dígitos hexadecimales en minúscula |
| QoS | 1 |
| `retain` | **false**, obligatorio |

`retain` debe ser false sin excepción. Un setpoint retenido en el broker se le entrega a
la placa apenas se reconecta, y la mandaría a una posición calculada para un instante que
ya pasó.

Con QoS 1 el broker puede reentregar un mensaje (`dup`). Eso es inofensivo acá: la regla
de puntos vencidos (§6.3) descarta lo que llega tarde.

> **Atención al depurar.** Existen dos tópicos terminados en `/coordinates/polar` y
> llevan cosas distintas:
>
> - `plugin/<plugin_uuid>/coordinates/polar` → **JSON**, `PolarCoordinatesPayload`, lo
>   publica `services/coord_transform`.
> - `device/<device_id>/coordinates/polar` → **binario**, el DTO de este documento.
>
> Un `mosquitto_sub` sobre el segundo muestra bytes ilegibles. Eso es lo esperado, no una
> falla. Usar `mosquitto_sub -F '%x'` para verlo en hexadecimal.

## 3. Formato del payload

Un mensaje es un header de longitud fija seguido de `count` puntos de 8 bytes.

    +----------------------+------------+------------+-----+
    |   HEADER (19 bytes)  | POINT[0]   | POINT[1]   | ... |
    +----------------------+------------+------------+-----+
                             8 bytes      8 bytes

**Tamaño total: `19 + 8 * count` bytes.** Todos los enteros son **little-endian** y no hay
padding entre campos: las estructuras se serializan empaquetadas. El ESP32 y el host del
servidor son ambos little-endian, así que no hay conversión de orden de bytes en ninguno
de los dos lados.

### 3.1 Header (19 bytes)

| offset | size | campo | tipo | valor / rango |
|---|---|---|---|---|
| 0 | 1 | `magic` | `uint8` | `0x50` (ASCII `'P'`) |
| 1 | 1 | `flags` | `uint8` | bit 0 = `HOLD`; bits 1-7 reservados, deben ser 0 |
| 2 | 1 | `count` | `uint8` | cantidad de puntos, 1 a 16. Sólo puede ser 0 si `HOLD` está seteado |
| 3 | 8 | `t0_ms` | `int64` | instante objetivo del punto 0, epoch UTC en milisegundos |
| 11 | 8 | `t_sent_ms` | `int64` | instante en que el servidor emitió el mensaje, epoch UTC en milisegundos |

### 3.2 Punto (8 bytes, repetido `count` veces)

| offset | size | campo | tipo | rango |
|---|---|---|---|---|
| 0 | 4 | `dt_ms` | `uint32` | offset en milisegundos desde `t0_ms` |
| 4 | 2 | `az_cdeg` | `uint16` | 0 … 35999, azimut en centigrados (0.01°) |
| 6 | 2 | `el_cdeg` | `int16` | −9000 … 9000, elevación en centigrados (0.01°) |

El instante objetivo del punto `i` es `t0_ms + dt_ms[i]`. El punto 0 normalmente lleva
`dt_ms = 0`, pero no está obligado a hacerlo.

### 3.3 Para qué sirve cada campo del header

- **`magic`** — primer byte fijo. Si algo publica JSON o basura en este tópico por error,
  la placa ve que no arranca con `0x50` y lo descarta. Sin este byte, bytes arbitrarios se
  interpretarían como ángulos y la antena se movería a cualquier lado.
- **`flags`** — bits de encendido/apagado que no son coordenadas. Hoy sólo `HOLD`
  (`0x01`), que le pide a la placa que detenga el movimiento. Los bits restantes quedan
  reservados y el decoder debe rechazar un mensaje que traiga alguno seteado, para que
  usarlos en el futuro no rompa en silencio contra una placa vieja.
- **`count`** — cuántos puntos vienen atrás. La placa lo necesita para saber dónde termina
  el mensaje y para validar el largo total.
- **`t0_ms`** — base temporal de la trayectoria.
- **`t_sent_ms`** — cuándo lo emitió el servidor. No participa del apuntamiento: es el
  único instrumento para medir la latencia de transporte, comparándolo contra la hora de
  llegada en la placa. Va una sola vez en el header, no por punto. Es lo que permite
  verificar el techo de 500 ms de RNF-01.2.

### 3.4 Por qué estas representaciones

- **Centigrados en vez de `float`** — 0.01° de resolución son cien veces más finos que la
  precisión angular de ±1° que el sistema se propone, y órdenes de magnitud por debajo de
  lo que un servo real resuelve. Ahorra 4 bytes por punto contra dos `float` y elimina
  cualquier discusión sobre coma flotante entre Python y C.
- **`int64` de milisegundos** — la resolución de milisegundos es la que pide RNF-08.1, el
  rango de 64 bits evita el problema del año 2038, y mapea directo a
  `int(dt.timestamp() * 1000)` en Python y a `int64_t` en C.
- **`dt_ms` relativo en vez de un timestamp absoluto por punto** — ahorra 4 bytes por
  punto. Un `uint32` de milisegundos cubre unos 49 días, muy por encima de cualquier
  ventana de trayectoria razonable.
- **Struct empaquetado en vez de JSON o CBOR** — la placa lo decodifica con un `memcpy`,
  sin heap, sin parser y sin agregar una dependencia al firmware. Es además el DTO binario
  que plantea RA-10.1.

## 4. Semántica de los timestamps

Los dos timestamps del header responden preguntas distintas y no son intercambiables:

- **`t0_ms` + `dt_ms` (objetivo)** — "la antena debe estar apuntando a (az, el) en este
  instante". Es lo que permite al servidor mandar la trayectoria por adelantado y a la
  placa ejecutarla a tiempo, absorbiendo el jitter de la red. Depende de que ambos relojes
  estén sincronizados por NTP; por eso §6.4 define qué pasa cuando no lo están.
- **`t_sent_ms` (emisión)** — "el servidor emitió este mensaje en este instante". Sirve
  para medir cuánto tardó en llegar, y para nada más.

## 5. Versionado

Este formato **no tiene un campo `version`**. Si cambia de forma incompatible, se cambia
el **valor de `magic`**: `0x51` para la v2, `0x52` para la v3, y así.

Esto no es un detalle opcional. Una placa que no fue reflasheada tiene que descartar un
payload de un formato que no conoce, en vez de leerlo con el layout equivocado y mover la
antena a una posición arbitraria. Como el decoder ya rechaza todo lo que no arranca con el
magic que conoce (§6.1), cambiar ese byte es lo único que hace falta — pero hay que
acordarse de hacerlo, porque si el formato cambia sin tocar `magic` el error es silencioso.

Cambios compatibles hacia atrás —usar un bit reservado de `flags`, por ejemplo— no
requieren cambiar `magic`.

## 6. Reglas de validación y comportamiento del decoder

### 6.1 Rechazo temprano

El decoder descarta el mensaje, lo loguea y sigue si se cumple cualquiera de estas:

- `magic != 0x50`
- algún bit reservado de `flags` (bits 1-7) está seteado
- `count > 16`
- `count == 0` y `HOLD` no está seteado
- `payload_len != 19 + 8 * count`

Un mensaje inválido nunca aborta, nunca reinicia y nunca mueve la antena. La validación de
largo es la que protege de leer fuera del buffer, así que va antes de tocar cualquier
punto.

### 6.2 Alineación

El payload se decodifica copiando con `memcpy` hacia una estructura
`__attribute__((packed))`, nunca casteando el puntero que entrega `esp-mqtt`: ese buffer no
tiene alineación garantizada, y un acceso de 32 o 64 bits sobre una dirección no alineada
es un error en Xtensa.

### 6.3 Puntos vencidos

Un punto cuyo instante objetivo ya pasó por más de una tolerancia configurable se descarta
en lugar de ejecutarse tarde: moverse a una posición calculada para hace varios segundos es
peor que no moverse. Si **todos** los puntos del batch están vencidos, se descarta el
mensaje completo.

### 6.4 Reloj sin sincronizar

Si `sntp_manager_is_synced()` devuelve false, los timestamps del mensaje no son comparables
con nada y toda la semántica temporal se cae. En ese caso el decoder **descarta el batch** y
la placa reporta el estado del reloj.

Se consideró la alternativa de ejecutar el último punto inmediatamente, como degradación
"mejor esfuerzo". Queda descartada: sin reloj no hay forma de saber si ese punto tiene un
segundo o diez minutos de antigüedad, y apuntar a una posición arbitraria es peor que
quedarse quieto y reportar el problema.

### 6.5 Orden de los puntos

Los `dt_ms` deben venir estrictamente crecientes. Un batch desordenado indica un bug del
lado del servidor y se descarta completo en vez de intentar reordenarlo.

### 6.6 `HOLD`

Con el bit 0 de `flags` seteado, el servidor pide detener el movimiento. El mensaje no
lleva puntos (`count == 0`) y mide 19 bytes. La placa frena y se queda donde está.

## 7. Ejemplo trabajado

Trayectoria de tres puntos, separados un segundo, emitida dos segundos antes del primer
objetivo:

    t0_ms     = 1789763400000        (2026-09-18T20:30:00.000Z)
    t_sent_ms = 1789763398000        (2 s antes)
    flags     = 0x00
    count     = 3

    punto 0:  dt =    0 ms,  az = 123.45°,  el = 45.00°
    punto 1:  dt = 1000 ms,  az = 124.00°,  el = 45.30°
    punto 2:  dt = 2000 ms,  az = 124.55°,  el = 45.60°

Bytes en el cable (43 en total):

    hdr   50 00 03 40 31 36 b6 a0 01 00 00 70 29 36 b6 a0 01 00 00
    pt0   00 00 00 00 39 30 94 11
    pt1   e8 03 00 00 70 30 b2 11
    pt2   d0 07 00 00 a7 30 d0 11

En una línea:

    500003403136b6a0010000702936b6a00100000000000039309411e80300007030b211d0070000a730d011

Un mensaje `HOLD` (19 bytes):

    50 01 00 40 31 36 b6 a0 01 00 00 70 29 36 b6 a0 01 00 00

### 7.1 Encoder de referencia (Python)

```python
import struct

MAGIC = 0x50
FLAG_HOLD = 0x01


def encode(t0_ms, t_sent_ms, points, flags=0):
    """points: iterable of (dt_ms, az_cdeg, el_cdeg)."""
    points = list(points)
    header = struct.pack("<BBBqq", MAGIC, flags, len(points), t0_ms, t_sent_ms)
    body = b"".join(struct.pack("<IHh", dt, az, el) for dt, az, el in points)
    return header + body
```

Conversión desde los grados en coma flotante que produce el servidor:

```python
az_cdeg = round(az_deg % 360.0 * 100)      # 0 .. 35999
el_cdeg = round(el_deg * 100)              # -9000 .. 9000
t_ms = int(dt.timestamp() * 1000)          # dt debe ser timezone-aware en UTC
```

### 7.2 Estructuras de referencia (C)

```c
#define PLUTO_DTO_MAGIC        0x50
#define PLUTO_DTO_FLAG_HOLD    0x01
#define PLUTO_DTO_MAX_POINTS   16
#define PLUTO_DTO_HEADER_LEN   19
#define PLUTO_DTO_POINT_LEN    8

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t flags;
    uint8_t count;
    int64_t t0_ms;
    int64_t t_sent_ms;
} pluto_dto_header_t;

typedef struct __attribute__((packed)) {
    uint32_t dt_ms;
    uint16_t az_cdeg;
    int16_t  el_cdeg;
} pluto_dto_point_t;
```

Ambas estructuras deben cumplir
`_Static_assert(sizeof(pluto_dto_header_t) == PLUTO_DTO_HEADER_LEN)` y
`_Static_assert(sizeof(pluto_dto_point_t) == PLUTO_DTO_POINT_LEN)`.

## 8. Tamaño máximo del mensaje

| `count` | bytes |
|---|---|
| 0 (`HOLD`) | 19 |
| 1 | 27 |
| 3 | 43 |
| **5** | **59** |
| 6 | 67 |

RNF-19.1 fija el payload de control idealmente por debajo de 64 bytes, y eso es lo único
que acota el batch a **5 puntos por mensaje**. No es una limitación del formato: `count`
admite hasta 16, lo que daría 147 bytes. Si hacen falta ventanas de trayectoria más largas,
el número a revisar es ese requisito —con un techo de 128 bytes entrarían 13 puntos—, no
el layout.

## 9. Pendiente de implementar

- Decoder en el firmware: `components/coord_dto` con `coord_dto_decode()`, enganchado en
  `coordinates_message_handler` de `main/main.c`.
- Encoder y dispatcher en el servidor: el puente de
  `plugin/<plugin_uuid>/coordinates/polar` a `device/<device_id>/coordinates/polar`.
- La tolerancia concreta de la regla 6.3, que depende de la dinámica de los motores.
- Vectores de prueba compartidos entre ambos repos. Se decidió no hacerlos por ahora; si
  aparece un desacuerdo de endianness o de escala entre las dos implementaciones, ése es el
  momento de agregarlos.
