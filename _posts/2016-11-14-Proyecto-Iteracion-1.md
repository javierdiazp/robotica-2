---
layout: post
title:  "Proyecto: Iteración 1"
date:   2016-11-14 14:00:00
author: Javier Díaz, Patricio Merino
categories: ROS
---

# Proyecto: Iteración 1

## Introducción
Como hemos visto en entradas anteriores, nuestro proyecto consiste en realizar mejoras a la línea de comando de ROS dado que, en el poco tiempo que hemos trabajado con ella, hemos observado varias tareas que podrían ser mejor facilitadas al usuario. 

Además existen algunos errores (bugs) que impiden usar debidamente los comandos y en general una serie de procesos cuya lógica creemos que debería ser modificada para ajustarse a la perspectiva de lo que espera un usuario novato.

Este blog se presenta en este contexto, donde nos damos un espacio para explicar los avances realizados en lo que denominamos la primera iteración (de dos) y donde además contamos nuestras experiencias trabajando en el código interno de ROS.

Dividiremos el blog en cuatro partes (incluyendo esta) donde trataremos abiertamente los tópicos que compenden al problema a nivel teórico y además hablaremos un poco acerca de cómo queremos hacer llegar nuestras ideas para una posible integración con la versión oficial de ROS.

Añadido a esto incluiremos una sección por cada parte de la solución del proyecto donde discutiremos sobre el diseño de esta y, brevemente, sobre la implementación práctica.

## Problema
Nuestro proyecto en realidad ataca varios problemas pequeños y localizados en lugar de intentar solucionar uno demasiado grande. Este podría ser descrito de forma general con la siguiente frase: "La línea de comandos que ofrece ROS presenta un conjunto de procedimientos sin la suficiente funcionalidad para adaptarse a nuevos usuarios", a lo que además agregamos que existen errores en ciertos comandos y lógicas que creemos que deberían ser indispensables pero que no están implementadas. 

Se procederá a describir cada sub-problema de modo que para efectos de este blog, las siguientes secciones se referirán a ellos por su número asignado. Además nos hemos encargado de ordenarlos en orden de menor a mayor dificultad.

1. Obtener información extra de un elemento en ROS se hace con el comando ros___ info, sin embargo `rosmsg info` no existe. En su lugar se nos propone `rosmsg show`.

2. La opción que un usuario espera en `rostopic pub` por default es `-1`, pero el default actúa diferente de esta.

3. El comando `rostopic pub` nos pide escribir cada vez el tipo de mensaje, pero este es estático sobre el tópico.

4. La función de autocompletar no sirve cuando `rostopic pub` lleva opciones.

5. Es difícil saber qué tópicos no tienen subscribers o publishers, ya que hay que comparar cada tópico en dos listas distintas (la de tópicos con subscribers y la de tópicos con publishers).

6. No se puede buscar tipos de mensajes dentro de otros tipos de mensajes, como por ejemplo buscar un `Float64` dentro de mensajes de tipo `Twist`.

7. Al cerrar `roscore` los tópicos asociados a este no mueren, lo que puede presentar fallas en la lógica esperada si se vuelve a abrir `roscore` y se hace referencia a tópicos de la sesión anterior.

8. No existe ninguna manera de añadir alguna explicación al momento de crear nodos y tópicos que permita entender al usuario el objetivo de estos. En su lugar se debe deducir la funcionalidad solamente del nombre.


## Parte 1: No existe rosmsg info

### Solución
Se propone la creación de `rosmsg info` simplemente como alias de `rosmsg show`, es decir, se desea mantener el método anterior por motivos de adaptabilidad de los antiguos usuarios.

### Diseño
Para esto se pretende agregar `rosmsg info` en el parser de `rosmsg` y simplemente redirigir la llamada a `rosmsg show`. Además se deben modificar las referencias asociadas como el help de `rosmsg` y la función de autocompletar, ambas equivalentes al comando original.

### Implementación
Se modificó el archivo `rosmsg/\__init__\.py` donde ahora el comando `info` es parseado como `show` con alias "info", lo que significa que es equivalente salvo por el help.

Se puede observar el código fuente [aquí](https://github.com/javierdiazp/ros_comm/tree/indigo-devel-rosmsg-info).

## Parte 2: El default de rostopic pub no es -1

### Aclaración
En primera instancia se pensaba modificar la opción por defecto a `-1`, sin embargo luego de una investigación un poco más profunda se determinó que la opción que está por defecto (latch) si tiene sentido.

### Solución
De acuerdo a esto, el nuevo objetivo pasa a ser la modificación de la opción `-1` de manera que actúe de acuerdo a la lógica esperada, es decir, que publique a los tópicos que están escuchando y que finalice inmediatamente (actualmente se esperan 3 segundos para terminar).

### Diseño
Para esto se busca eliminar el delay que existe en el comando y modificar el mensaje de ejecución para indicar que finaliza inmediatamente en lugar de exponer el tiempo.

### Implementación
Se modificó el archivo `rostopic/\__init__\.py` cambiando el tiempo de ejecución del comando a una cantidad indiscernible para el ser humano y se imprime en pantalla que la publicación finaliza de inmediato. Esto se realizó así dado que el comando necesita un delay explícito para ser efectivo (con delay 0 no se publica el mensaje).

Se puede observar el código fuente [aquí](https://github.com/javierdiazp/ros_comm/tree/indigo-devel-rostopic-pub-option-1).

## Parte 3: Se exige tipo de mensaje en rostopic pub

### Solución 1
Se propone que el comando infiera el tipo cuando este no es descrito explícitamente. Esto ocurrirá siempre que el tópico tenga un tipo asociado, es decir, que haya sido creado previamente.

### Diseño 1
Para conseguir esto se planea utilizar la información entregada por el comando rostopic info, de modo que cuando se necesita inferir el tipo de un tópico, este sea reemplazado a nivel de parser por el substring de info que lo contiene.

### Implementación 1
En la práctica hay una consideración que vuelve ambigua la posibilidad de omitir el tipo y es que los argumentos del mensaje son descritos de la misma manera que los argumentos del comando. Debido a esto, cuando la cantidad de palabras es grande se vuelve arbitrario decidir cual de ellas corresponde a tópico, mensaje o argumentos del mensaje si además se debe lidiar con el hecho de que puede haber o no haber tipo.


### Solución 2
Se propone la funcionalidad de autocompletar el tipo de mensaje directamente el tipo estático del tópico en lugar de con una lista con todos los tipos posibles. De este modo, si al presionar tab el tópico ya había sido definido, se rellena automáticamente el comando con la información inferida.

### Diseño 2
Para esto se pretende modificar el autocompletado con tab de manera que para rellenar el tipo existan dos opciones. Si el tópico no ha sido creado aún se mantiene el comportamiento inicial (una lista con todos los mensajes); en cambio si el tópico ya es conocido se autocompleta directamente con su tipo estático.

### Implementación 2
Se modificó el archivo `share/rosbash/rosbash` el cual contiene las funciones de autocompletado que son llamadas por la terminal cuando son requeridas. Dicho archivo está en el lenguaje bash el cual representa una dificultad extra a la hora de realizar las modificaciones.
El cambio en cuestión consistió en preguntar por la existencia anterior del tópico, evitando imprimir el mensaje de error si no existía, para finalmente agregar la nueva opción a partir del tipo entregado por `rostopic info`.

Se puede observar el código fuente [aquí](https://github.com/javierdiazp/ros/tree/indigo-devel-pub-autocomplete-type).

## Parte 4: Autocompletar de rostopic pub no sirve con opciones

### Solución
Modificar la función de autocompletado para permitir su uso con opciones.

### Diseño
Para esto se pretende parsear la parte del comando escrito hasta el momento y determinar las palabras que corresponden al tópico, el tipo y el mensaje, ignorando las opciones. De este modo se reutilizan las funciones anteriores, pero a partir de los nuevos índices determinados.

### Implementación
Una consideración a tener en cuenta es que la opción `-r` lleva un argumento, por lo que es parseado de forma especial. Además se contabilizan los índices de manera que se correspondan a la manera en que se hacía en el bash, lo que permite comparar si la palabra actual corresponde o no a un caso de autocompletado.

Se puede observar el código fuente [aquí](https://github.com/javierdiazp/ros/tree/indigo-devel-rostopic-pub-autocomplete-with-options).

## Parte 5: Difícil saber qué tópicos no tienen subscribers o publishers

### Solución 1
Se propone unificar las listas de publishers y subscribers bajo alguna nueva opción. Además 

### Diseño 1
El diseño aún no está lista para esta iteración.

### Implementación 1
La implementación aún no está lista para esta iteración.


### Solución 2
Se propone una opción para mostrar la lista de tópicos que no tienen subscribers o bien publishers, denominados tópicos zombies ya que no realizan trabajo útil.

### Diseño 2
El diseño aún no está lista para esta iteración.

### Implementación 2
La implementación aún no está lista para esta iteración.


## Parte 6: No se puede buscar subtipos de mensajes

### Solución
Se propone una opción para `rostopic find` que permita la búsqueda recursiva entre tipos de mensajes

### Diseño
El diseño aún no está lista para esta iteración.

### Implementación
La implementación aún no está lista para esta iteración.


## Parte 7: Al cerrar roscore los tópicos asociados no mueren

### Solución
Se propone corregir el comportamiento de roscore para que al menos se manejen adecuadamente los tópicos zombies al cerrarse.

### Diseño
El diseño aún no está lista para esta iteración.

### Implementación
La implementación aún no está lista para esta iteración.


## Parte 8: Los nodos y tópicos no tienen una descripción asociada

### Solución
Se propone una opción para agregar un mensaje de descripción al momento de crear un nodo o un tópico, y que este sea visible a partir del comando `ros___ info`.

### Diseño
El diseño aún no está lista para esta iteración.

### Implementación
La implementación aún no está lista para esta iteración.

## Integración con ROS
Después de una investigación sobre los requisitos necesarios para realizar una petición de integración a la versión oficial de ROS obtuvimos que se deben cumplir ciertas normativas en lo que respecta al código y al formato de la propuesta. 

En primera instancia se requiere que el trabajo sea manejado en GitHub, y se presente a través un pull request con un commit por cada feature. 

Además el formato del título debe indicar como sufijo el nombre del proyecto y el mensaje debe ser suficientemente clarificador respecto de lo que propone la nueva funcionalidad.

Lo anterior conlleva a la observación de que el método de subdividir nuestro proyecto en pequeñas tareas mejore la adaptabilidad al formato requerido por ROS y facilite la construcción de commits, con lo cual es posible presentar todas las funcionalidades nombradas por separado y aumentar la posibilidad de que alguna de ellas sea catalogada como útil y finalmente aceptada.

## Conclusión
En el corto plazo observamos modificaciones a ROS que realmente aportan en el desempeño de usuarios tanto nuevos como antiguos, principalmente los asociados al comando `rostopic pub`.

Estas modificaciones muestran ser candidatas interesantes para ser presentadas a una versión oficial de ROS debido a que son modificaciones pequeñas de la funcionalidad, pero hacen más grato el trabajo en este sistema.

Con respecto al método de trabajo utilizado, que se basa en tareas pequeñas pero progresivas, creemos que se adaptan en gran medida con la forma en que se presenta nuestro proyecto, de modo que podemos concentrarnos en problemas más acotados y directos. Esto resulta en una ventaja dado que evita que el esfuerzo decaiga con cada jornada.

A largo plazo creemos que los límites impuestos en nuestra [Definición de intenciones](https://javierdiazp.github.io/robotica2/ros/2016/10/13/Proyecto-Definicion-de-intenciones.html) van en buen camino y son plausibles de realizar durante la realización de este ramo.