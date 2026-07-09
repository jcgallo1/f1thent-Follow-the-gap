# F1TENTH Hybrid Reactive Controller (ROS2)

Controlador híbrido para vehículos autónomos F1TENTH desarrollado en **ROS2**, que combina navegación mediante **Waypoints**, **Follow the Gap** y **Follow Wall Assist** para completar un circuito de manera autónoma, rápida y segura.

El proyecto fue desarrollado como una mejora al algoritmo clásico **Follow the Gap**, incorporando navegación basada en objetivos (Waypoints), control adaptativo de velocidad y asistencia de seguimiento de paredes para aumentar la estabilidad en escenarios con obstáculos.

---

# Objetivo

El objetivo principal del controlador es completar una vuelta al circuito siguiendo una trayectoria óptima previamente definida mediante waypoints.

A diferencia de un algoritmo puramente reactivo, el vehículo no decide su trayectoria únicamente observando el LiDAR. En este proyecto existe una trayectoria objetivo, y únicamente cuando dicha trayectoria se encuentra bloqueada el vehículo cambia temporalmente a un comportamiento reactivo para evitar obstáculos.

De esta manera se consigue un comportamiento más natural, estable y eficiente.

---

# Problema

Existen dos enfoques ampliamente utilizados en F1TENTH.

## Follow the Gap

Follow the Gap analiza el LiDAR y selecciona continuamente el hueco libre más amplio para avanzar.

Su principal ventaja es que evita obstáculos de forma muy eficiente.

Sin embargo, presenta varias limitaciones:

- No conoce la pista.
- Puede escoger trayectorias poco eficientes.
- En curvas puede desviarse innecesariamente.
- Cambia constantemente de dirección dependiendo del LiDAR.

---

## Navegación mediante Waypoints

La navegación por waypoints consiste en definir una secuencia de coordenadas que representan la trayectoria ideal del circuito.

El vehículo únicamente intenta llegar al siguiente waypoint.

Su principal ventaja es que mantiene una trayectoria muy eficiente.

No obstante, tiene un inconveniente importante:

Si aparece un obstáculo inesperado, el vehículo continuará intentando llegar al waypoint aunque exista riesgo de colisión.

---

## Solución propuesta

Este proyecto combina ambos enfoques.

Los **Waypoints** representan el objetivo principal.

**Follow the Gap** solamente se activa cuando el camino hacia dicho objetivo se encuentra bloqueado.

Finalmente, **Follow Wall Assist** ayuda a estabilizar el vehículo cuando circula muy cerca de una pared durante una maniobra evasiva.

De esta forma se obtiene un controlador híbrido que mantiene la trayectoria ideal siempre que sea posible y únicamente modifica su comportamiento cuando es realmente necesario.

---

# Arquitectura general

La siguiente imagen resume el comportamiento implementado durante una vuelta completa del circuito.

![Arquitectura del controlador](assets/referenciaProyecto.png)

La idea general es sencilla:

1. El vehículo identifica el siguiente waypoint.
2. El LiDAR analiza el camino hacia dicho waypoint.
3. Si el camino está libre continúa normalmente.
4. Si existe un obstáculo se activa Follow the Gap.
5. Si además existe una pared cercana se activa Follow Wall Assist.
6. Una vez superado el obstáculo el controlador vuelve automáticamente al seguimiento de waypoints.

Todo este proceso ocurre continuamente durante cada iteración del nodo de ROS2.

---

# Funcionamiento del controlador

El controlador se ejecuta como un nodo de ROS2 llamado:

```
unified_hybrid_racer
```

Este nodo recibe información de:

- `/scan` (LaserScan)
- `/ego_racecar/odom` (Odometry)

y publica los comandos de conducción mediante:

```
/drive
```

En cada iteración realiza el siguiente proceso.

---

## Paso 1. Lectura de sensores

El controlador obtiene continuamente dos fuentes principales de información.

### LiDAR

El LiDAR proporciona la distancia a los obstáculos alrededor del vehículo.

Toda la toma de decisiones depende de esta información.

Con el LiDAR el vehículo puede determinar:

- espacio libre
- obstáculos
- paredes
- distancia frontal
- distancia lateral

---

### Odometría

La odometría proporciona:

- posición X
- posición Y
- orientación (Yaw)

Estos datos permiten conocer exactamente dónde se encuentra el vehículo dentro del circuito.

---

## Paso 2. Selección del siguiente Waypoint

Los waypoints representan la trayectoria ideal del circuito.

El controlador mantiene una lista ordenada de coordenadas.

Cada waypoint posee la forma:

```
(x,y)
```

Durante toda la carrera el vehículo intenta alcanzar el waypoint actual.

Cuando la distancia al waypoint es menor a aproximadamente **3.5 metros**, automáticamente cambia al siguiente.

Esto permite recorrer el circuito completo de forma continua.

---

## Paso 3. Calcular la dirección objetivo

Una vez conocido el waypoint actual, el controlador calcula el ángulo necesario para llegar hasta él.

Esta operación se realiza mediante la función:

```
compute_goal_angle()
```

El resultado representa la dirección ideal del vehículo.

Si no existieran obstáculos, ésta sería exactamente la dirección utilizada para conducir.

---

## Paso 4. Analizar el camino

Antes de girar hacia el waypoint, el LiDAR analiza si realmente existe un camino libre.

Para ello se revisan principalmente dos regiones.

### Zona frontal

Determina la distancia libre justo delante del vehículo.

---

### Zona del waypoint

Analiza únicamente la región donde se encuentra el waypoint.

De esta forma el controlador responde la siguiente pregunta:

**¿Existe un camino libre hacia el objetivo?**

---

## Paso 5. Determinar el estado del entorno

Dependiendo de la distancia detectada por el LiDAR, el controlador clasifica el entorno en cuatro estados.

| Estado | Distancia | Comportamiento |
|----------|------------|----------------|
| SAFE | > 4.5 m | Velocidad máxima |
| CAUTION | 1.5 – 4.5 m | Reduce velocidad |
| AVOID | Camino bloqueado | Activa Follow the Gap |
| DANGER | < 1.35 m | Máxima protección |

Estos estados permiten adaptar automáticamente la velocidad y el comportamiento del vehículo.

---

# Navegación por Waypoints

Este es el modo principal del controlador.

Mientras el camino permanezca libre:

- no busca huecos
- no sigue paredes
- únicamente conduce hacia el waypoint

Este comportamiento produce una trayectoria mucho más estable que un algoritmo completamente reactivo.

---

# Follow the Gap

Cuando el LiDAR detecta que el camino hacia el waypoint está bloqueado, el controlador cambia automáticamente al modo **AVOID**.

En este momento comienza el algoritmo Follow the Gap.

Su funcionamiento consta de cuatro etapas.

---

## 1. Crear una burbuja

El obstáculo más cercano se rodea mediante una burbuja virtual.

Todos los rayos del LiDAR contenidos dentro de esa burbuja son descartados.

Esto evita que el vehículo intente atravesar el obstáculo.

---

## 2. Buscar huecos

Una vez eliminada la zona ocupada por el obstáculo, el algoritmo analiza todos los espacios libres restantes.

Cada conjunto continuo de rayos libres se considera un posible camino.

---

## 3. Evaluar cada hueco

No todos los huecos son igualmente buenos.

Cada uno recibe una puntuación considerando varios criterios.

Entre ellos:

- cercanía al waypoint
- distancia disponible
- posición dentro del hueco
- continuidad respecto a la dirección anterior

Gracias a esto el vehículo evita cambios bruscos de dirección.

---

## 4. Seleccionar el mejor hueco

Finalmente se escoge el hueco con mayor puntuación.

La dirección del vehículo cambia hacia dicho hueco hasta que el camino vuelve a quedar libre.

---

# Follow Wall Assist

Durante algunas maniobras evasivas el vehículo circula muy cerca de una pared.

En estas situaciones únicamente seguir el hueco puede producir pequeñas oscilaciones.

Para solucionarlo se implementó un asistente de seguimiento de pared.

Este módulo:

- utiliza mediciones laterales del LiDAR
- estima la distancia al muro
- calcula una pequeña corrección en el volante

Es importante destacar que este módulo **no conduce el vehículo**.

Únicamente añade una pequeña corrección para estabilizar la trayectoria.

---

# Control adaptativo de velocidad

La velocidad depende directamente del espacio libre detectado por el LiDAR.

Cuando el camino está completamente despejado:

- velocidad máxima

Cuando aparecen obstáculos:

- velocidad media

Cuando el riesgo aumenta:

- velocidad baja

Cuando existe riesgo de colisión:

- velocidad mínima

Esto permite que el vehículo sea rápido cuando el entorno es seguro y conservador únicamente cuando es necesario.

---

# Flujo completo del algoritmo

Durante cada iteración del nodo ocurre exactamente el siguiente proceso.

```
Inicio

↓

Leer LiDAR

↓

Leer odometría

↓

Calcular siguiente waypoint

↓

Calcular ángulo hacia el waypoint

↓

Analizar camino frontal

↓

¿Camino libre?

├── Sí
│
│   Seguir Waypoint
│
└── No
    │
    Aplicar burbuja
    │
    Buscar huecos
    │
    Seleccionar mejor hueco
    │
    Aplicar Follow Wall Assist (si es necesario)

↓

Calcular velocidad

↓

Publicar AckermannDrive

↓

Repetir
```

Todo este proceso ocurre continuamente mientras el simulador está en ejecución.

---

# Organización del proyecto

```
reactive_race/

├── reactive_race/
│
│   ├── raceline_follower.py
│   ├── raceline_obs.py
│   └── ...
│
├── package.xml
├── setup.py
├── setup.cfg
├── run_follower.sh
└── README.md
```

---

## raceline_follower.py

Es el controlador principal del vehículo.

Implementa:

- lectura del LiDAR
- lectura de odometría
- seguimiento de waypoints
- Follow the Gap
- Follow Wall Assist
- control de velocidad
- control de dirección
- cronómetro de vueltas
- publicación de comandos Ackermann

---

## raceline_obs.py

Controla el segundo robot utilizado como obstáculo dinámico durante las pruebas.

Su objetivo es generar escenarios donde el controlador deba reaccionar ante tráfico en movimiento.

---

## setup.py

Registra ambos nodos para ROS2.

```python
'raceline_follower = reactive_race.raceline_follower:main',
'raceline_obs = reactive_race.raceline_obs:main',
```

---

# Instrucciones de instalación y ejecución

## 1. Copiar el mapa

Dentro del repositorio se encuentra la carpeta:

```
Mapa/

SaoPaulo_mapObs.png
SaoPaulo_mapObs.yaml
```

Copiar ambos archivos al directorio de mapas utilizado por el simulador F1TENTH.

---

## 2. Configurar el Launch

Modificar el archivo de lanzamiento correspondiente para:

- actualizar la ruta (`path`) del nuevo mapa.
- seleccionar `SaoPaulo_mapObs.yaml`.
- agregar un segundo robot que actuará como obstáculo dinámico.

---

## 3. Verificar los ejecutables

En `setup.py` deben existir los siguientes entry points:

```python
'raceline_follower = reactive_race.raceline_follower:main',

'raceline_obs = reactive_race.raceline_obs:main',
```

---

## 4. Compilar el paquete

```bash
colcon build
```

---

## 5. Cargar el workspace

```bash
source install/setup.bash
```

---

## 6. Ejecutar el simulador

Iniciar el simulador F1TENTH utilizando el mapa configurado previamente.

---

## 7. Ejecutar el vehículo principal

```bash
ros2 run reactive_race raceline_follower
```
 
---

## 8. Ejecutar el robot dinámico

En otra terminal ejecutar:

```bash
ros2 run reactive_race raceline_obs
```

---

# Tecnologías utilizadas

- ROS2
- Python
- NumPy
- F1TENTH Simulator
- Ackermann Steering
- LaserScan
- Odometry

---

# Conclusiones

El controlador desarrollado combina las ventajas de la navegación deliberativa y reactiva mediante una arquitectura híbrida. Mientras los waypoints proporcionan una trayectoria óptima y estable, Follow the Gap permite reaccionar ante obstáculos inesperados y Follow Wall Assist mejora la estabilidad cerca de las paredes.

Esta integración permite completar el circuito de forma autónoma manteniendo altas velocidades cuando el entorno es seguro y reduciendo el riesgo de colisiones cuando aparecen obstáculos, logrando un equilibrio entre eficiencia, estabilidad y seguridad.