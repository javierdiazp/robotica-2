---
layout: post
title:  "Proyecto: Informe Final"
date:   2017-01-02 22:00:00
author: Javier Díaz, Patricio Merino
categories: ROS
---

# Proyecto: Informe Final

## Introducción
Esta es la iteración final del proyecto y abarca tanto a las propuestas de integración con ROS de las modificaciones realizadas en la primera iteración, así como las nuevas modificaciones realizadas exclusivamente en la segunda.z

La estructura de este post se distribuye de manera que abarca cada una de las dos mitades mencionadas con sus respectivos mini-proyectos, detallando en cada sección sobre el problema en cuestión, la solución propuesta inicialmente y la implementación de la solución final. En la primera mitad se entrega un resumen de lo detallado en el post específico de la iteración 1, además de una descripción sobre los resultados del intento de integración a ROS.

Cabe añadir que, si bien en la segunda parte sólo se implementaron dos soluciones, también se realizó el trabajo de publicar los Pull Request de toda la primera mitad lo que conllevó a una serie de modificaciones a los códigos principalmente en el área de los help y la forma en que se presentan los resultados.

## Primera parte

### Proyecto 1: rosmsg info

Ver código fuente [aquí](https://github.com/javierdiazp/ros_comm/tree/indigo-devel-rosmsg-info).

#### Problema
Obtener información extra de un elemento en ROS se hace con el comando `ros[--] info`, sin embargo rosmsg info no existe. En su lugar ROS propone el uso de `rosmsg show` lo que en particular es inconsistente con el resto de comandos y puede provocar confusión en los nuevos usuarios.

#### Solución propuesta
Agregar `rosmsg info` en el parser de `rosmsg` y simplemente redirigir la llamada a `rosmsg show`. Además se debe tomar en cuenta que hay que modificar las referencias asociadas como el help de `rosmsg` y la función de autocompletar, y definirlas ambas equivalentes al comando original.

#### Solución final
Se modificó el parser de la función `rosmsg` de la manera propuesta, pero además se integró un identificador del alias para implementar el help.

#### Integración con ROS
Un Pull Request con la modificación fue presentado al repositorio oficial [ros_comm](https://github.com/ros/ros_comm) el 29 de noviembre y fue aprobado el mismo día para su integración a la próxima versión de ROS Indigo.


### Proyecto 2: rostopic pub - repeticiones por defecto

Ver código fuente [aquí](https://github.com/javierdiazp/ros_comm/tree/indigo-devel-rostopic-pub-option-1).

#### Problema
La opción `-1` de rostopic pub tarda 3 segundos en terminar el proceso en lugar de hacerlo de inmediato. Además si el usuario realmente quiere que el proceso no termine inmediatamente, puede usar la opción por defecto (latch) que deja el proceso abierto hasta que se cancele a mano.

#### Solución propuesta
Modificar la opción `-1` de para que actúe de acuerdo a la lógica esperada, es decir, que publique a los tópicos que están escuchando y que finalice inmediatamente.

#### Solución final
Se cambió el delay de término del comando `rostopic pub -1` a 0.1 segundos, dejando la opción por defecto (sin el `-1`) inalterada.

#### Integración con ROS
Un Pull Request con la modificación fue presentado al repositorio oficial [ros_comm](https://github.com/ros/ros_comm) el 29 de noviembre. Los test de building automáticos pasaron satisfactoriamente, pero aún no se ha decidido si la propuesta será integrada o descartada.


### Proyecto 3: rostopic pub - autocompletar tipo de mensaje

Ver código fuente [aquí](https://github.com/javierdiazp/ros/tree/indigo-devel-pub-autocomplete-type).

#### Problema
El comando rostopic pub pide escribir cada vez el tipo de mensaje del tópico, pero este es estático, es decir, para un tópico definido, el tipo de mensaje es único.

#### Solución propuesta
Utilizar la información entregada por el comando rostopic info, de modo que cuando se necesita inferir el tipo de un tópico, este sea reemplazado a nivel de parser por el substring de info que lo contiene.

#### Solución final
Se modificó el archivo `share/rosbash/rosbash` el cual contiene las funciones de autocompletado que son llamadas por la terminal cuando son requeridas. Dicho archivo está en el lenguaje bash el cual representó una dificultad extra a la hora de realizar las modificaciones.
El cambio en cuestión consistió en preguntar por la existencia anterior del tópico, evitando imprimir el mensaje de error si no existía, para finalmente agregar la nueva opción a partir del tipo entregado por `rostopic info`.

#### Integración con ROS
Un Pull Request con la modificación fue presentado al repositorio oficial [ros](https://github.com/ros/ros) el 29 de noviembre. Los test de building automáticos pasaron satisfactoriamente, pero aún no se ha decidido si la propuesta será integrada o descartada.


### Proyecto 4: rostopic pub - autocompletar con opciones

Ver código fuente [aquí](https://github.com/javierdiazp/ros/tree/indigo-devel-rostopic-pub-autocomplete-with-options).

#### Problema
La función de autocompletar no sirve cuando rostopic pub lleva opciones

#### Solución propuesta
Modificar el parser del comando escrito hasta el momento y determinar las palabras que corresponden al tópico, el tipo y el mensaje, para finalmente llamar a seguir con la función ya existente, pero empleando estos nuevos índices.

#### Solución final
Igual que para el proyecto 3, se modificó el archivo `share/rosbash/rosbash`. Se utilizaron variables para mantener los índices de las palabras clave del comando, para así parsearlo correctamente. Además hay un caso particular para la opción `-r` (ya que esta lleva argumentos) el cual se resolvió como un caso aparte.
El cambio en cuestión consistió en preguntar por la existencia anterior del tópico, evitando imprimir el mensaje de error si no existía, para finalmente agregar la nueva opción a partir del tipo entregado por `rostopic info`.

#### Integración con ROS
Un Pull Request con la modificación fue presentado al repositorio oficial [ros](https://github.com/ros/ros) el 29 de noviembre. Los test de building automáticos pasaron satisfactoriamente, pero aún no se ha decidido si la propuesta será integrada o descartada.


## Segunda Parte

### Proyecto 5: rostopic list - verbose unificado

Ver código fuente [aquí](https://github.com/javierdiazp/ros_comm/tree/indigo-devel-verbose2).

#### Problema
Es difícil saber qué tópicos no tienen subscribers o publishers, ya que hay que comparar cada tópico en dos listas distintas, que además no mantienen ningún orden especificado. Por otra parte, la existencia de estas dos lista conlleva a mantener información duplicada en pantalla.

#### Solución propuesta
Crear una nueva opción verbose2 con las listas de subscribers y publishers unificadas y ordenadas.

#### Solución final
Se emplea mergesort para juntar y ordenar ambas listas de manera eficiente. Además se modifica el orden en el que aparece la información para que el número de subscribers y de publishers de cada tópico aparezca al principio, lo que facilita la visualzación.

#### Integración con ROS
Un Pull Request con la modificación fue presentado al repositorio oficial [ros_comm](https://github.com/ros/ros_comm) el 29 de noviembre. Los test de building automáticos pasaron satisfactoriamente, pero aún no se ha decidido si la propuesta será integrada o descartada.



### Proyecto 6: rostopic find - opción recursiva

Ver código fuente [aquí](https://github.com/javierdiazp/ros_comm/tree/indigo-devel-recursive-find).

#### Problema
No se puede filtrar los tópicos a partir de tipos de mensajes que estén contenidos en otros tipos de mensaje. Esto implica que no se pueden conocer realmente todos los tópicos asociados a un tipo de mensaje.

#### Solución propuesta
Implementar una opción para rostopic find que permita la búsqueda recursiva entre tipos de mensajes

#### Solución final
La opción agregada (`-r` o `--recursive`) utiliza la información entregada por los paquetes de ROS para buscar en el contenido de cada tipo de mensaje. Esto permite realizar la búsqueda sobre cada subtipo, lo que se hace de manera eficiente manteniendo un diccionario que recuerda si cierto tipo X contiene en algún sub-nivel a cierto tipo Y, de modo que no se repitan el número de verificaciones.
Incluso así la búsqueda recursiva resulta ser un proceso considerablemente más lento que la opción por defecto (del orden de algunos pocos segundos), debido a que en cada ejecución del comando se debe computar sobre ciertos grupos completos de paquetes de mensajes, al contrario de la búsqueda normal donde la información está almacenada directamente por el master de ROS.

#### Integración con ROS
No se realizó un Pull Request porque el comando, si bien era funcional, requería de algunas mejoras en optimalidad para adecuarse al estándar del resto de los comandos en cuanto a tiempo de procesamiento.


## Tareas no completadas

### Optimización de la búsqueda recursiva de rostopic find
Una de las dificultades encontradas durante el desarrollo de esta solución es que el master de ROS no parece tener información directa sobre los elementos contenidos en los tipos de mensajes, de manera que se necesitó investigar el funcionamiento del sistema de paquetes de ROS para generar una solución que se internara en los subtipos de cada mensaje.
Debido a que la información no es directa y se debe computar con cada comando, el resultado es un delay de unos cuantos segundos antes de que se imprima en pantalla la información requerida, lo que en realidad no suficientemente aceptable para un comando de línea de carácter ligero como lo es el de ROS.

### Corregir el funcionamiento anómalo de roscore al cerrarse
Si bien se realizaron ciertas hipótesis sobre la causa del problema, fundada en nuestras experiencias durante el desarrollo con roscore, no se logró alcanzar a iniciar ningún diseño preliminar sobre un posible resultado para este problema.

### Implementar la opción de agregar una breve descripción durante la creación de nodos y tópicos
Por falta de tiempo no se logro comenzar con este proyecto.

## Conclusión

Todos los Pull Request realizados a los repositorios oficiales de ROS pasaron los test de building aunque sólo uno fue aceptado realmente para su integración con ROS, incluso durante el mismo día de su publicación. 
Esto deja la sensación de que aquellas modificaciones más simples de integrar y que resuelven problemas sencillos parecen ser los que tienen una mayor probabilidad de ser considerados.

Si bien no todas las tareas declaradas en nuestra [Definición de intenciones](https://javierdiazp.github.io/robotica2/ros/2016/10/13/Proyecto-Definicion-de-intenciones.html) fueron completadas, esto se debe a que nos topamos con diversos problemas a la hora de implementar que requirieron de cierta investigación extendida, como por ejemplo el funcionamiento del sistema de paquetes de ROS.

Finalmente no dejamos de considerar aceptable el resultado de que al menos una de nuestras soluciones resultó lo suficientemente útil para los usuarios como para que esta fuera aprobada para su integración en un sistema de uso tan extendido a nivel internacional como lo es ROS.



