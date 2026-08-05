# manipulation_planner

Nodo puente entre los consumidores de manipulacion de Justina y MoveIt.

Este paquete no ejecuta motores directamente. Recibe una pose objetivo,
construye una solicitud de planeacion para MoveIt y devuelve la trayectoria
articular resultante a quien la pidio.

## Que hace ahora

El nodo principal es `manipulation_planner_node`.

Expone el servicio:

* `/manipulation/manipulation_planner/plan_trajectory`

Flujo de planeacion:

1. Recibe una `target_pose`, el `planning_group` y el `tip_link`.
2. Valida que el `tip_link` corresponda al grupo solicitado.
3. Lee el estado articular actual desde la fuente canonica:
   * `/hardware/joint_states`
4. Prepara, si esta habilitado, el filtro de nube dedicado a manipulacion.
5. Construye una solicitud `GetMotionPlan` para MoveIt.
6. Fuerza la planeacion con:
   * `pipeline_id = ompl`
   * `planner_id = RRTConnect`
7. Convierte la respuesta de MoveIt a listas planas de `joint_names` y
   `waypoints`.

## Fuente canonica de joints

La fuente canonica del estado articular es:

* `/hardware/joint_states`

Antes el planner escuchaba `/manipulation/joint_states`. Eso podia crear una
segunda verdad sobre la posicion real del robot. Ahora el topic se controla con
el parametro:

* `joint_states_topic`

El launch lo deja por defecto en `/hardware/joint_states`.

## Planner usado

La ruta de ejecucion real usa OMPL con RRTConnect:

* `planning_pipeline_id = ompl`
* `planner_id = RRTConnect`

Esto se manda explicitamente en cada solicitud a MoveIt. Por eso, aunque alguna
vista del Setup Assistant muestre otro valor en la GUI, la evidencia runtime
debe buscarse en los logs de `move_group`, por ejemplo:

* `Using planning pipeline 'ompl'`
* `right_arm_group[RRTConnect]`
* `geometric::RRTConnect`

## Tolerancias de goal

Las tolerancias de MoveIt se configuran por parametros:

* `goal_position_tolerance_m`
* `goal_orientation_tolerance_rad`

Estas tolerancias evitan que una pose de vision demasiado exacta bloquee IK o
sampling cuando una pose cercana tambien seria valida para el agarre.

## Filtro de nube y objeto objetivo

Cuando `use_cloud_filter` esta activo, el planner prepara una ventana corta de
planeacion:

1. configura la exclusion del objeto objetivo;
2. activa el filtro de nube;
3. limpia el octomap de MoveIt;
4. espera un breve `warmup`;
5. pide el plan a MoveIt;
6. desactiva el filtro al terminar.

En simulacion se puede lanzar con `use_cloud_filter:=false` para probar la
cadena de manipulacion sin depender de Kinect ni octomap.
